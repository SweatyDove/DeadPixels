#include <iostream>
#include <vector>
#include <array>
#include <string>

#include <cmath>


#include <opencv2/opencv.hpp>


cv::Point2i findNeighbour(cv::Point2i target, cv::Mat image, int direction);


#define     MAX_DISTANCE                    2       // Расстояние от битого пикселя до ближайшего исправного (в любом направлении, просто: X ---> Y )
#define     DISTANCE_DIFFERENCE_THRESHOLD   6       // Расстояние между исправными пикселями, т.е.: Y1 <--- X ---> Y2;  X - битый пиксель, Y - исправный
#define     INTENCITY_LEVEL                 128


#define     mcr_PASTE(front, back)   front ## back


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

//using deadAndCo = std::array<float, DeadAndCo::TOTAL>;


// # Структура описывает координаты битого пикселя и его ближайших исправных соседей
//class Neighbors {
//public:

//    // # Мертвый пиксель
//    cv::Point2i dead {};

//    // # Горизонталь
//    cv::Point2i left {};
//    cv::Point2i right {};

//    // # Вертикаль
//    cv::Point2i top {};
//    cv::Point2i bottom {};

//    // # Диагональ 1
//    cv::Point2i topLeft {};
//    cv::Point2i bottomRight {};

//    // # Диагональ 2
//    cv::Point2i bottomLeft {};
//    cv::Point2i topRight {};

//    Neighbors(int x, int y) :
//        dead {x, y},
//        left {x, y},
//        right {x, y},
//        top {x, y},
//        bottom {x, y},
//        topLeft {x, y},
//        bottomRight {x, y},
//        bottomLeft {x, y},
//        topRight {x, y}
//    {

//    }
//};


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

std::vector<std::array<int, 14>>&& formCorrectionData(std::vector<std::array<cv::Point2i, Direction::TOTAL>>& inputData);




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
    std::vector<std::array<int, 14>> outputData {std::move(formCorrectionData(deadAndCo))};

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









//        float topRightDistance = std::sqrt(static_cast<float>(std::pow(pixel.dead.x - pixel.topRight.x, 2) + std::pow(pixel.dead.y - pixel.topRight.y, 2)));
//        float topLeftDistance = std::sqrt(static_cast<float>(std::pow(pixel.dead.x - pixel.topLeft.x, 2) + std::pow(pixel.dead.y - pixel.topLeft.y, 2)));
//        float rightDistance = std::sqrt(static_cast<float>(std::pow(pixel.dead.x - pixel.right.x, 2) + std::pow(pixel.dead.y - pixel.right.y, 2)));
//        float leftDistance = std::sqrt(static_cast<float>(std::pow(pixel.dead.x - pixel.left.x, 2) + std::pow(pixel.dead.y - pixel.left.y, 2)));
//        float topDistance = std::sqrt(static_cast<float>(std::pow(pixel.dead.x - pixel.top.x, 2) + std::pow(pixel.dead.y - pixel.top.y, 2)));
//        float bottomDistance = std::sqrt(static_cast<float>(std::pow(pixel.dead.x - pixel.bottom.x, 2) + std::pow(pixel.dead.y - pixel.bottom.y, 2)));
//        float bottomRightDistance = std::sqrt(static_cast<float>(std::pow(pixel.dead.x - pixel.bottomRight.x, 2) + std::pow(pixel.dead.y - pixel.bottomRight.y, 2)));
//        float bottomLeftDistance = std::sqrt(static_cast<float>(std::pow(pixel.dead.x - pixel.bottomLeft.x, 2) + std::pow(pixel.dead.y - pixel.bottomLeft.y, 2)));


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


////        return 0;

//    }






    return 0;
}


//==================================================================================================
// Загрузить изображение
//==================================================================================================
void downloadImage()
{


    // # Конструкция для отображения изображения в памяти НЕПРЕРЫВНО


}


//==================================================================================================
// Строим карту битых пикселей
//==================================================================================================
void getDeadMap(void* image)
{
    /*
     * 1) Проходим по каждому пикселю, проверяем его интенсивность @Int (градация серого > 50%).
     *
     * 2) Записываем каждый найденный пиксель в std::array<float, 14> - пока только координаты,
     *      остальные параметры в ноль. Формируем вектор из таких пикселей. Или же просто записать
     *      только x|y. Или сразу записать ВООБЩЕ всё - это, наверное простой вариант самый. А потом
     *      можно доработать через запись только строк, ближайших к пикселю и корректных. Или вообще
     *      как-то подобрать квадрат с границами по всем направлениям.
     *
     * 3) Затем для каждого пикселя находим ближайших исправных соседей (сперва горизонталь-вертикаль,
     *      потом диагонали).
     *      a) Сперва нам нужна линия (горизонталь/вертикаль) с разницей < MAX_DISTANCE между небитыми
     *      пикселями. Затем нужно проверить расстояние от центра до исправного пикселя по каждой линии -
     *      должно быть < DISTANCE_DIFFERENCE_THRESHOLD
     *
     *       Если есть 4 соседа - то расчитываем интенсивность в точке по формуле:
     *      Int = w1 * Int1 + w2 * Int2 + w3 * Int3 + w4 * Int4.
     *      Выше w - весовые коэффициенты. Если используются 2 линии - то по 0.5f на линию, если одна - то 1.0f
     *
     * 4) Если нет горизонтали/вертикали, то работаем с диагоналями. Если и их нет - то просто с ближайшими
     *      соседями (я бы тут увеличил вклад соседа, расположенного ближе
     */
}


//==================================================================================================
// Скорректировать изображение с учётом карты битых пикселей
//==================================================================================================
void imageCorrection(void* image, void* deadMap)
{
    /*
     * 1) Есть физические координаты экрана (пикселей на мониторе, возможно, пикселей в каком-то окне) в OpenGL. Нам важно, как их видит
     *  OpenGL - центр, это (0.0f, 0.0f); левый нижний угол - это (-1.0f, -1.0f); правый верхний - это (1.0f, 1.0f).
     *  Как он преобразует эти числа в координаты пикселей на мониторе - это нам не важно.
     *
     * 2) Есть текстурные координаты: они от (0.0f, 0.0f) до (1.0f, 1.0f) - это координаты именно текстуры (изображения),
     *      которое нужно отобразить на экране (как я понимаю). Ещё тут нужно учесть смещение в пол пикселя (разберусь уже на деле)
     *
     * 3) Вывод на экран за счёт cv::imshow().
     *
     *
     */
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



class Line {
public:
    int mb_left {-1};
    int mb_right {-1};
    int mb_length {-1};

    bool isValid {false};

    Line(int a, int b) :
        mb_left {std::abs(a)},
        mb_right {std::abs(b)},
        mb_length {mb_left + mb_right}
    {
        // Nothing to do
    }

    int getLength() { return mb_length; }
};





class LineDiag {
private:
    cv::Point2i mb_left {};
    cv::Point2i mb_dead {};
    cv::Point2i mb_right {};

    int mb_maxDistance {};
    int mb_distDiffThreshold {};

public:

    bool isValid {false};

    LineDiag(cv::Point2i left, cv::Point2i dead, cv::Point2i right, int maxDistance, int distDiffThreshold)  :
        mb_left {left},
        mb_dead {dead},
        mb_right {right},
        mb_maxDistance {maxDistance},
        mb_distDiffThreshold {distDiffThreshold}
    {
        // 1) Понять, можно ли использовать линию в принципе
        if (this->checkDistance()) {

        }


    }


    // # Проверка на расстояние от мертвого пикселя до соседа
    bool checkDistance()
    {
        if ((std::sqrt(std::pow((mb_left.x - mb_dead.x), 2) + std::pow((mb_left.y - mb_dead.y), 2)) < mb_maxDistance) &&
            (std::sqrt(std::pow((mb_right.x - mb_dead.x), 2) + std::pow((mb_righ.y - mb_dead.y), 2)) < mb_maxDistance)) {
            return true;
        }
        else {
            return false;
        }
    }


    int getLength() { return mb_length; }
};



//==================================================================================================
// Функция формирует список пикселей, по которым будет идти корректировка битого пикселя:
//==================================================================================================
std::vector<std::array<int, 14>>&& formCorrectionData(std::vector<std::array<cv::Point2i, Direction::TOTAL>>& inputData)
{
    std::vector<std::array<int, 14>> outputData (inputData.size());

    bool foundCoorPix {true};



    // # Нужно найти 4 ближайших точки, удовлетворяющих алгоритму
    for (int ii {0}; ii < inputData.size(); ++ii) {

        std::array<cv::Point2i, Direction::TOTAL> pixel {inputData[ii]};

        std::vector<cv::Point2i> corrPoints {};

        // # Координаты битого пикселя
        outputData[ii][X0] = inputData[ii][DEAD].x;
        outputData[ii][Y0] = inputData[ii][DEAD].y;



        // # Сперва проверяем пиксели по горизонтали и по вертикали на условие, что расстояние от
        // # битого до исправного пикселя < MAX_DISTANCE
        Line horizontal {pixel[DEAD].x - pixel[LEFT].x, pixel[RIGHT].x - pixel[DEAD].x};
        Line vertical {pixel[TOP].y - pixel[DEAD].y, pixel[DEAD].y - pixel[BOTTOM].y};

        if ((horizontal.mb_left > 0 && horizontal.mb_left < MAX_DISTANCE)
            && (horizontal.mb_right > 0 && horizontal.mb_right < MAX_DISTANCE)) {
            horizontal.isValid = true;
        }
        else {}

        if ((vertical.mb_left > 0 && vertical.mb_left < MAX_DISTANCE)
            && (vertical.mb_right > 0 && vertical.mb_right < MAX_DISTANCE)) {
            vertical.isValid = true;
        }
        else {}


        // # Теперь смотрим на длину линий
        if (horizontal.isValid == true && vertical.isValid == true) {

            // # Какая-то из линий (может ОБЕ) удовлетворяют условию - используем такие линии (точки)
            if (horizontal.getLength() < DISTANCE_DIFFERENCE_THRESHOLD) {
                corrPoints.push_back(pixel[LEFT]);
                corrPoints.push_back(pixel[RIGHT]);
            }
            else {}

            if (vertical.getLength() < DISTANCE_DIFFERENCE_THRESHOLD) {
                corrPoints.push_back(pixel[TOP]);
                corrPoints.push_back(pixel[BOTTOM]);
            }
            else {}


            // # Обе линии НЕ удовлетворяют условию - используем только точки меньшей линии
            if (horizontal.getLength() > DISTANCE_DIFFERENCE_THRESHOLD && vertical.getLength() > DISTANCE_DIFFERENCE_THRESHOLD) {
                if (horizontal.getLength() <= vertical.getLength()) {
                    corrPoints.push_back(pixel[LEFT]);
                    corrPoints.push_back(pixel[RIGHT]);
                }
                else {
                    corrPoints.push_back(pixel[TOP]);
                    corrPoints.push_back(pixel[BOTTOM]);
                }
            }
        }
        // # Только горизонтальная валидна
        else if (horizontal.isValid == true) {
            corrPoints.push_back(pixel[LEFT]);
            corrPoints.push_back(pixel[RIGHT]);
        }
        // # Только вертикальная валидна
        else if (vertical.isValid == true) {
            corrPoints.push_back(pixel[TOP]);
            corrPoints.push_back(pixel[BOTTOM]);
        }
        // # Никакая из горизонтальных/вертикальных линий не валидна - переходим к диагоналям
        else {
            foundCoorPix = false;
        }

        // # Если достаточно гориз/верт линий - то заполняем соответствующими точками наш массив
        // # и переходим к следующей точке
        if (foundCoorPix == true) {
            for (int kk {0}, id {2}; kk < corrPoints.size(); ++kk, id += 2) {
                outputData[ii][id] = corrPoints[kk].x;
                ++id;
                outputData[ii][id] = corrPoints[kk].y;
            }
            continue;
        }
        else {
            foundCoorPix = true;        // Для аналогичной работы следующей части
        }



        // # Теперь учитываем диагональные линии
        LineDiag backward {pixel[TOP_LEFT], pixel[DEAD], pixel[BOTTOM_RIGHT], MAX_DISTANCE, DISTANCE_DIFFERENCE_THRESHOLD};
        LineDiag forward {pixel[BOTTOM_LEFT], pixel[DEAD], pixel[TOP_RIGHT], MAX_DISTANCE, DISTANCE_DIFFERENCE_THRESHOLD};

    }
}








