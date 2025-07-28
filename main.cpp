#include <iostream>
#include <vector>
#include <array>
#include <string>

#include <cmath>


#include <opencv2/opencv.hpp>




#define     MAX_DISTANCE                    2       // Расстояние от битого пикселя до ближайшего исправного (в любом направлении, просто: X ---> Y )
#define     DISTANCE_DIFFERENCE_THRESHOLD   6       // Расстояние между исправными пикселями, т.е.: Y1 <--- X ---> Y2;  X - битый пиксель, Y - исправный
#define     INTENCITY_LEVEL                 128




enum OutputData {
    X0,
    Y0,

    X1,
    Y1,
    W1,

    X2,
    Y2,
    W2,

    X3,
    Y3,
    W3,

    X4,
    Y4,
    W4,
};





enum Direction {
    TOP_LEFT,
    TOP,
    TOP_RIGHT,

    LEFT,
    DEAD,
    RIGHT,

    BOTTOM_LEFT,
    BOTTOM,
    BOTTOM_RIGHT,

    TOTAL

};




struct Line {
    cv::Point2i startPoint {};
    cv::Point2i endPoint {};
};



cv::Point2i findNeighbour(cv::Point2i target, cv::Mat image, int direction);
void formCorrectionData(std::vector<std::array<cv::Point2i, Direction::TOTAL>>& inputData, std::vector<std::array<int, 14>>& outputData);
std::vector<cv::Point2i>* checkLine(const Line& lineAA, const Line& lineBB, const cv::Point2d& deadPoint, float maxDistance, float distDiffThreshold);
void addFactors(std::array<int, 14>& outputData, std::vector<cv::Point2i>* correctionPoints);





int main()
{
    const std::string imagePath {"/home/alexey/Programming/Projects/Qt/DeadPixel/Task/test.png"};
    std::vector<std::array<cv::Point2i, Direction::TOTAL>> deadAndCo {};



    cv::Mat image {cv::imread(imagePath, cv::IMREAD_GRAYSCALE)};
//    if (image.isContinuous() == false) {
//        image = image.clone();
//    }
//    else {}
    const int imageWidth {image.cols};
    const int imageHeight {image.rows};



    cv::namedWindow("Original", cv::WINDOW_KEEPRATIO);
    cv::imshow("Original", image);
    cv::resizeWindow("Original", 300, 200);
    cv::waitKey(0);





    // # Ищем битые пиксели
    for (int row {0}; row < imageHeight; ++row) {
        for (int column {0}; column < imageWidth; column++) {
            if (image.at<unsigned char>(cv::Point(column, row)) >= INTENCITY_LEVEL) {
                std::array<cv::Point2i, 9> temp { {
                    {column, row},          // BOTTOM_LEFT
                    {column, row},          // BOTTOM
                    {column, row},          // BOTTOM_RIGHT
                    {column, row},          // LEFT
                    {column, row},          // DEAD
                    {column, row},          // RIGHT
                    {column, row},          // TOP_LEFT
                    {column, row},          // TOP
                    {column, row}           // TOP_RIGHT
                }
                };
                deadAndCo.push_back(temp);
            }
            else {}
        }
    }


    // # Выводим список битых пикселей в формате (столбец, строка) и ищем соседей для каждого из них.
    // # Можно искать не всех соседей сразу, а сперва по горизонтали/вертикали - нечетные направления.
    for (auto& pixel: deadAndCo) {
//        std::cout << '(' << pixel[DEAD].y + 1 << "; " << pixel[DEAD].x + 1 << ')' << std::endl;

        for (int direction {TOP_LEFT}; direction <= BOTTOM_RIGHT; ++direction) {
            pixel[direction] = findNeighbour(pixel[DEAD], image, direction);
        }
    }



    // # Теперь заполняем выходной массив, согласно алгоритму
    std::vector<std::array<int, 14>> outputData;
    formCorrectionData(deadAndCo, outputData);

    for (auto& data: outputData) {
        std::cout << '['
                  << data[0] << ", "
                  << data[1] << ", "
                  << data[2] << ", "
                  << data[3] << ", "
                  << data[4] << ", "
                  << data[5] << ", "
                  << data[6] << ", "
                  << data[7] << ", "
                  << data[8] << ", "
                  << data[9] << ", "
                  << data[10] << ", "
                  << data[11] << ", "
                  << data[12] << ", "
                  << data[13] << ')'
                  << std::endl;
    }


    return 0;
}





//==================================================================================================
// Поиск ближайшего к @target небитого пикселя
//==================================================================================================
cv::Point2i findNeighbour(cv::Point2i target, cv::Mat image, int direction)
{
    const int deadColumn {target.x};
    const int deadRow {target.y};
    const int imageWidth {image.cols};
    const int imageHeight {image.rows};

    cv::Point2i retPoint {target.x, target.y};

    switch (direction) {
        case BOTTOM_LEFT:
        for (int column {deadColumn}, row {deadRow}; (column >= 0) && (row < imageHeight); --column, ++row) {
            if (image.at<unsigned char>(row, column) < INTENCITY_LEVEL) {
                retPoint.x = column;
                retPoint.y = row;
                break;
            }
            else {}
        }
        break;
        case BOTTOM:
            for (int row {deadRow}; row < imageHeight; ++row) {
                if (image.at<unsigned char>(row, deadColumn) < INTENCITY_LEVEL) {
                   retPoint.y = row;
                    break;
                }
                else {}
            }
        break;
        case BOTTOM_RIGHT:
            for (int column {deadColumn}, row {deadRow}; (column < imageWidth) && (row < imageHeight); ++column, ++row) {
                if (image.at<unsigned char>(row, column) < INTENCITY_LEVEL) {
                    retPoint.x = column;
                    retPoint.y = row;
                    break;
                }
                else {}
            }
        break;
        case LEFT:
            for (int column {deadColumn}; column >= 0; --column) {
                if (image.at<unsigned char>(deadRow, column) < INTENCITY_LEVEL) {
                    retPoint.x = column;
                    break;
                }
                else {}
            }
        break;
        case DEAD:
        break;
        case RIGHT:
            for (int column {deadColumn}; column < imageWidth; ++column) {
                if (image.at<unsigned char>(deadRow, column) < INTENCITY_LEVEL) {
                    retPoint.x = column;
                    break;
                }
                else {}
            }
        break;
        case TOP_LEFT:
            for (int column {deadColumn}, row {deadRow}; (column >= 0) && (row >= 0); --column, --row) {
                if (image.at<unsigned char>(row, column) < INTENCITY_LEVEL) {
                    retPoint.x = column;
                    retPoint.y = row;
                    break;
                }
                else {}
            }
        break;
        case TOP:
            for (int row {deadRow}; row >= 0; --row) {
                if (image.at<unsigned char>(row, deadColumn) < INTENCITY_LEVEL) {
                    retPoint.y = row;
                    break;
                }
                else {}
            }
        break;
        case TOP_RIGHT:
        for (int column {deadColumn}, row {deadRow}; (column < imageWidth) && (row >= 0); ++column, --row) {
            if (image.at<unsigned char>(row, column) < INTENCITY_LEVEL) {
                retPoint.x = column;
                retPoint.y = row;
                break;
            }
            else {}
        }
        break;
    }

    return retPoint;


}






//==================================================================================================
// Функция формирует список пикселей, по которым будет идти корректировка битого пикселя:
//==================================================================================================
void formCorrectionData(std::vector<std::array<cv::Point2i, Direction::TOTAL>>& inputData, std::vector<std::array<int, 14>>& outputData)
{

    // # Нужно найти 4 ближайших точки, удовлетворяющих алгоритму
    for (int ii {0}; ii < inputData.size(); ++ii) {

        const std::array<cv::Point2i, Direction::TOTAL>& pixel {inputData[ii]};

        // # Координаты битого пикселя
        outputData[ii][X0] = inputData[ii][DEAD].x;
        outputData[ii][Y0] = inputData[ii][DEAD].y;


        /*******************************************************************************************
         * Обернуть в умный указатель?
         ******************************************************************************************/
        std::vector<cv::Point2i>* correctionPoints {};
        std::vector<float>* correctionFactors {};

        // # Сперва проверяем пиксели по горизонтали и вертикали
        const Line horizontal {pixel[LEFT], pixel[RIGHT]};
        const Line vertical {pixel[TOP], pixel[BOTTOM]};

        correctionPoints = checkLine(horizontal, vertical, pixel[DEAD], MAX_DISTANCE, DISTANCE_DIFFERENCE_THRESHOLD);

        // # Если достаточно гориз/верт линий - то заполняем соответствующими точками наш массив
        // # и переходим к следующей точке
        if (correctionPoints->size() > 0) {
            for (int kk {0}, id {2}; kk < correctionPoints->size(); ++kk, id += 2) {
                outputData[ii][id] = correctionPoints->at(kk).x;
                ++id;
                outputData[ii][id] = correctionPoints->at(kk).y;
            }

            // Заполнение весовых коэффициентов
            addFactors(outputData[ii], correctionPoints);
            continue;
        }
        else {
            // # Данных недостаточно - смотрим другие линии
        }



        // # Теперь учитываем диагональные линии (с небольшой корректировкой MAX_DISTANCE)
        const Line forward {pixel[BOTTOM_LEFT], pixel[TOP_RIGHT]};
        const Line backward {pixel[TOP_LEFT], pixel[BOTTOM_RIGHT]};

        correctionPoints = checkLine(forward, backward, pixel[DEAD], MAX_DISTANCE * sqrt(2.0) , DISTANCE_DIFFERENCE_THRESHOLD);


        // # Если достаточно диагональных линий - то заполняем соответствующими точками наш массив
        // # и переходим к следующей точке
        if (correctionPoints->size() > 0) {
            for (int kk {0}, id {2}; kk < correctionPoints->size(); ++kk, id += 2) {
                outputData[ii][id] = correctionPoints->at(kk).x;
                ++id;
                outputData[ii][id] = correctionPoints->at(kk).y;
            }
            // Заполнение весовых коэффициентов
            addFactors(outputData[ii], correctionPoints);
            continue;
        }
        else {
            // # Данных недостаточно - используем ближайшие точки по горизонтали/вертикали
        }





        // # Используем ближайшие точки по горизонтали/вертикали
        int facPoints {0};
        int id {2};
        for (int direct {TOP}; direct <= BOTTOM; direct += 2) {
            // # Ближайшей точки в указанном направлении нет в принципе (граница или всё битое)
            if ((pixel[DEAD].x == pixel[direct].x) && (pixel[DEAD].y == pixel[direct].y)) {
                continue;
            }
            else {
                outputData[ii][id] = pixel[direct].x;
                ++id;
                outputData[ii][id] = pixel[direct].y;
                id += 2;
                ++facPoints;
            }
        }

        // # Заполнение весовых коэффициентов
        if (facPoints == 0) {
            // ## Просто пропускаем этот пиксель или кидаем исключение
            // throw std::runtime_error("Do not have enough data to correct image.")
        }
        else {
            float factor {1.0 / facPoints};
            id = W1;
            while (id < 14 && facPoints > 0) {
                outputData[ii][id] = factor;
                id += 3;
                --facPoints;
            }
        }
    } // for-loop



}




//==================================================================================================
// Функция проверяет точки двух переданных линий на удовлетворение условиям алгоритма. Возвращает
// от 0 до 4х точек.
//==================================================================================================
std::vector<cv::Point2i>* checkLine(const Line& lineAA, const Line& lineBB, const cv::Point2d& deadPoint, float maxDistance, float distDiffThreshold)
{
    std::vector<cv::Point2i>* retPoints {};

    bool lineAAValid {false};
    float lineAALength {0.0};

    float lineBBLength {0.0};
    bool lineBBValid {false};

    float startDeadLenght {0.0};
    float endDeadLenght {0.0};



    // # Проверка точек линии АА
    startDeadLenght = std::sqrt(std::pow((lineAA.startPoint.x - deadPoint.x), 2) + std::pow((lineAA.startPoint.y - deadPoint.y), 2));
    endDeadLenght = std::sqrt(std::pow((lineAA.endPoint.x - deadPoint.x), 2) + std::pow((lineAA.endPoint.y - deadPoint.y), 2));

    if ((startDeadLenght != 0.0 && endDeadLenght != 0.0) && (startDeadLenght <= maxDistance) && (endDeadLenght <= maxDistance)) {
        lineAAValid = true;
        lineAALength = startDeadLenght + endDeadLenght;
    }
    else {}

    // # Проверка точек линии BB
    startDeadLenght = std::sqrt(std::pow((lineBB.startPoint.x - deadPoint.x), 2) + std::pow((lineBB.startPoint.y - deadPoint.y), 2));
    endDeadLenght = std::sqrt(std::pow((lineBB.endPoint.x - deadPoint.x), 2) + std::pow((lineBB.endPoint.y - deadPoint.y), 2));

    if ((startDeadLenght != 0.0 && endDeadLenght != 0.0) && (startDeadLenght <= maxDistance) && (endDeadLenght <= maxDistance)) {
        lineBBValid = true;
        lineBBLength = startDeadLenght + endDeadLenght;
    }
    else {}



    // # Теперь сравниваем линии между собой
    if (lineAAValid == true && lineBBValid == true) {

        // ## Если какая-то из линий (может ОБЕ) удовлетворяют условию - используем точки этой линии
        if (lineAALength < distDiffThreshold) {
            retPoints->push_back(lineAA.startPoint);
            retPoints->push_back(lineAA.endPoint);
        }
        else {}

        if (lineBBLength < distDiffThreshold) {
            retPoints->push_back(lineBB.startPoint);
            retPoints->push_back(lineBB.endPoint);
        }
        else {}


        // ## Обе линии НЕ удовлетворяют условию - используем только точки меньшей линии
        if (lineAALength >= distDiffThreshold && lineBBLength >= distDiffThreshold) {

            /***************************************************************************************
             * В случае, когда lineAALength == lineBBLength, просто использую одну из двух.
             **************************************************************************************/
            if (lineAALength <= lineBBLength) {
                retPoints->push_back(lineAA.startPoint);
                retPoints->push_back(lineAA.endPoint);
            }
            else {
                retPoints->push_back(lineBB.startPoint);
                retPoints->push_back(lineBB.endPoint);
            }
        }
        else {}
    }
    // # Только линия АА валидна
    else if (lineAAValid == true) {
        retPoints->push_back(lineAA.startPoint);
        retPoints->push_back(lineAA.endPoint);
    }
    // # Только линия BB валидна
    else if (lineBBValid == true) {
        retPoints->push_back(lineBB.startPoint);
        retPoints->push_back(lineBB.endPoint);
    }
    // # Никакая из линий не валидна
    else {}


    return retPoints;
}



//==================================================================================================
// Получение весовых коэффициентов точек линии. Тут непонятно - мне считать относительно линии вес
// или же относительно кол-ва точек?
//==================================================================================================
void addFactors(std::array<int, 14>& outputData, std::vector<cv::Point2i>* correctionPoints)
{
    float sum {0.0};
    const float WEIGHT_2 {1.0};
    const float WEIGHT_4 {0.5};

    // # Одна линия используется (2 точки)
    if (correctionPoints->size() == 2) {
        for (int ii {0}, w {W1}; ii < correctionPoints->size(); ++ii) {
            float length = std::sqrt(std::pow(correctionPoints->at(ii).x - outputData[X0], 2) + std::pow(correctionPoints->at(ii).y - outputData[Y0], 2));
            outputData[w] = length;
            w += 3;
            sum += 1.0 / length;
        }

        float temp {WEIGHT_2 / sum};
        for (int w {W1}; w <= W2; w += 3) {
            outputData[w] = temp / outputData[w];
        }
    }
    // # Две линии используется (4 точки)
    else if (correctionPoints->size() == 4) {
        // ## Первая линия
        for (int ii {0}, w {W1}; ii < 2; ++ii) {
            float length = std::sqrt(std::pow(correctionPoints->at(ii).x - outputData[X0], 2) + std::pow(correctionPoints->at(ii).y - outputData[Y0], 2));
            outputData[w] = length;
            w += 3;
            sum += 1.0 / length;
        }

        float temp {WEIGHT_4 / sum};
        for (int w {W1}; w <= W2; w += 3) {
            outputData[w] = temp / outputData[w];
        }


        // ## Вторая линия
        sum = 0.0;
        for (int ii {2}, w {W3}; ii < correctionPoints->size(); ++ii) {
            float length = std::sqrt(std::pow(correctionPoints->at(ii).x - outputData[X0], 2) + std::pow(correctionPoints->at(ii).y - outputData[Y0], 2));
            outputData[w] = length;
            w += 3;
            sum += 1.0 / length;
        }

        temp = WEIGHT_4 / sum;
        for (int w {W3}; w <= W4; w += 3) {
            outputData[w] = temp / outputData[w];
        }
    }

}










////==================================================================================================
//// Строим карту битых пикселей
////==================================================================================================
//void getDeadMap(void* image)
//{
//    /*
//     * 1) Проходим по каждому пикселю, проверяем его интенсивность @Int (градация серого > 50%).
//     *
//     * 2) Записываем каждый найденный пиксель в std::array<float, 14> - пока только координаты,
//     *      остальные параметры в ноль. Формируем вектор из таких пикселей. Или же просто записать
//     *      только x|y. Или сразу записать ВООБЩЕ всё - это, наверное простой вариант самый. А потом
//     *      можно доработать через запись только строк, ближайших к пикселю и корректных. Или вообще
//     *      как-то подобрать квадрат с границами по всем направлениям.
//     *
//     * 3) Затем для каждого пикселя находим ближайших исправных соседей (сперва горизонталь-вертикаль,
//     *      потом диагонали).
//     *      a) Сперва нам нужна линия (горизонталь/вертикаль) с разницей < MAX_DISTANCE между небитыми
//     *      пикселями. Затем нужно проверить расстояние от центра до исправного пикселя по каждой линии -
//     *      должно быть < DISTANCE_DIFFERENCE_THRESHOLD
//     *
//     *       Если есть 4 соседа - то расчитываем интенсивность в точке по формуле:
//     *      Int = w1 * Int1 + w2 * Int2 + w3 * Int3 + w4 * Int4.
//     *      Выше w - весовые коэффициенты. Если используются 2 линии - то по 0.5f на линию, если одна - то 1.0f
//     *
//     * 4) Если нет горизонтали/вертикали, то работаем с диагоналями. Если и их нет - то просто с ближайшими
//     *      соседями (я бы тут увеличил вклад соседа, расположенного ближе
//     */
//}


////==================================================================================================
//// Скорректировать изображение с учётом карты битых пикселей
////==================================================================================================
//void imageCorrection(void* image, void* deadMap)
//{
//    /*
//     * 1) Есть физические координаты экрана (пикселей на мониторе, возможно, пикселей в каком-то окне) в OpenGL. Нам важно, как их видит
//     *  OpenGL - центр, это (0.0f, 0.0f); левый нижний угол - это (-1.0f, -1.0f); правый верхний - это (1.0f, 1.0f).
//     *  Как он преобразует эти числа в координаты пикселей на мониторе - это нам не важно.
//     *
//     * 2) Есть текстурные координаты: они от (0.0f, 0.0f) до (1.0f, 1.0f) - это координаты именно текстуры (изображения),
//     *      которое нужно отобразить на экране (как я понимаю). Ещё тут нужно учесть смещение в пол пикселя (разберусь уже на деле)
//     *
//     * 3) Вывод на экран за счёт cv::imshow().
//     *
//     *
//     */
//}




////==================================================================================================
//// Загрузить изображение
////==================================================================================================
//void downloadImage()
//{


//    // # Конструкция для отображения изображения в памяти НЕПРЕРЫВНО


//}


//        // # Теперь, в зависимости от дальности исправного соседа до битого пикселя, формируем значения
//        // # весов этих пикселей
//        float sum =   ((leftDistance == 0.0) ? 0.0 : 1.0 / leftDistance)
//                    + ((rightDistance == 0.0) ? 0.0 : 1.0 / rightDistance)
//                    + ((topDistance == 0.0) ? 0.0 : 1.0 / topDistance)
//                    + ((bottomDistance == 0.0) ? 0.0 : 1.0 / bottomDistance)
//                    + ((topLeftDistance == 0.0) ? 0.0 : 1.0 / topLeftDistance)
//                    + ((bottomRightDistance == 0.0) ? 0.0 : 1.0 / bottomRightDistance)
//                    + ((bottomLeftDistance == 0.0) ? 0.0 : 1.0 / bottomLeftDistance)
//                    + ((topRightDistance == 0.0) ? 0.0 : 1.0 / topRightDistance);

//        if (sum == 0.0) {
//            throw std::runtime_error("All pixels dead...");
//        }
//        else {
//            weightFactor = 1.0 / sum;
//        }


//        // # Теперь находим веса каждого соседа
//        float leftWeight = (leftDistance == 0.0) ? 0.0 : weightFactor / leftDistance;
//        float rightWeight = (rightDistance == 0.0) ? 0.0 : weightFactor / rightDistance;
//        float topWeight = (topDistance == 0.0) ? 0.0 : weightFactor / topDistance;
//        float bottomWeight = (bottomDistance == 0.0) ? 0.0 : weightFactor / bottomDistance;
//        float topLeftWeight = (topLeftDistance == 0.0) ? 0.0 : weightFactor / topLeftDistance;
//        float bottomRightWeight = (bottomRightDistance == 0.0) ? 0.0 : weightFactor / bottomRightDistance;
//        float bottomLeftWeight = (bottomLeftDistance == 0.0) ? 0.0 : weightFactor / bottomLeftDistance;
//        float topRightWeight = (topRightDistance == 0.0) ? 0.0 : weightFactor / topRightDistance;



//        // # Считаем интенсивности соседей
//        float leftIntencity = image.at<unsigned char>(pixel.left.y, pixel.left.x);
//        float rightIntencity = image.at<unsigned char>(pixel.right.y, pixel.right.x);
//        float topIntencity = image.at<unsigned char>(pixel.top.y, pixel.top.x);
//        float bottomIntencity = image.at<unsigned char>(pixel.bottom.y, pixel.bottom.x);
//        float topLeftIntencity = image.at<unsigned char>(pixel.topLeft.y, pixel.topLeft.x);
//        float bottomRightIntencity = image.at<unsigned char>(pixel.bottomRight.y, pixel.bottomRight.x);
//        float bottomLeftIntencity = image.at<unsigned char>(pixel.bottomLeft.y, pixel.bottomLeft.x);
//        float topRightIntencity = image.at<unsigned char>(pixel.topRight.y, pixel.topRight.x);


//        // # Исправляем битый пиксель
//        image.at<unsigned char>(deadRow, deadColumn) =   leftIntencity * leftWeight
//                                                       + rightIntencity * rightWeight
//                                                       + topIntencity * topWeight
//                                                       + bottomIntencity * bottomWeight
//                                                       + topLeftIntencity * topLeftWeight
//                                                       + bottomRightIntencity * bottomRightWeight
//                                                       + bottomLeftIntencity * bottomLeftWeight
//                                                       + topRightIntencity * topRightWeight;


//        cv::namedWindow("Original", cv::WINDOW_KEEPRATIO);
//        cv::imshow("Original", image);
//        cv::resizeWindow("Original", 300, 200);
//        cv::waitKey(0);





