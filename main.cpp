#include <iostream>
#include <iomanip>
#include <vector>
#include <array>
#include <string>

#include <cmath>
#include <cstdlib>


#include <opencv2/opencv.hpp>





//#define     MAX_DISTANCE                    2       // Расстояние от битого пикселя до ближайшего исправного (в любом направлении, просто: X ---> Y )
//#define     DISTANCE_DIFFERENCE_THRESHOLD   6       // Расстояние между исправными пикселями, т.е.: Y1 <--- X ---> Y2;  X - битый пиксель, Y - исправный
#define     INTENCITY_LEVEL                 128



// # Перечисляемый тип для удобства обращения к параметрам выходных данных
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




// # Перечисляемый тип для удобства обращения к соседям битого (DEAD) пикселя по направлениям
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



// # Просто структура для хранения линии
struct Line {
    cv::Point2i startPoint {};
    cv::Point2i endPoint {};
};



cv::Point2i findNeighbour(cv::Point2i target, cv::Mat image, int direction);
void formCorrectionData(std::vector<std::array<cv::Point2i, Direction::TOTAL>>& inputData, std::vector<std::array<float, 14>>& outputData);
void checkLine(const Line& lineAA, const Line& lineBB, const cv::Point2d& deadPoint, float maxDistance, float distDiffThreshold, std::vector<cv::Point2i>& retPoints);
void addFactors(std::array<float, 14>& outputData, std::vector<cv::Point2i>& correctionPoints);



/***************************************************************************************************
 *  Плохой вариант - потом убрать глобальные переменные
 **************************************************************************************************/
int MAX_DISTANCE {0};
int DISTANCE_DIFFERENCE_THRESHOLD {0};




int main(int argc, const char* argv[])
//int main()
{
    // # Debug
    //    int debug_argc {4};
    //    char* debug_argv[5] = {"deadpixels", "test.png", "2", "6", ""};



//    std::string imagePath {};
    std::string pixelMapPath {"pixelMap.png"};

//    int maxDistance {0};
//    int distanceDifferenceThreshold {0};


    // # Случай, когда пользователь указал некоректное число аргументов
    if (argc != 4) {
        std::cout << "\nUsage: DeadPixels <path-to-image> <max-distance> <distance-difference-threshold>" << std::endl;
        return 0;
    }
    else {
//        imagePath = argv[1];

        std::string maxDistanceStr {argv[2]};
        MAX_DISTANCE = std::atoi(maxDistanceStr.c_str());

        std::string distanceDifferenceThresholdStr {argv[3]};
        DISTANCE_DIFFERENCE_THRESHOLD = std::atoi(distanceDifferenceThresholdStr.c_str());
    }


    std::vector<std::array<cv::Point2i, Direction::TOTAL>> deadAndCo {};


    cv::Mat image {cv::imread(argv[1], cv::IMREAD_GRAYSCALE)};
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




    // # Ищем битые пиксели и заполняем массив соседей для каждого (много памяти, но проще потом работать)
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


    // # Ищем соседей для каждого битого пикселя. Можно искать не всех соседей сразу, а сперва
    // # по горизонтали/вертикали, потом по диагонали, если потребуется. Но пока сразу всех.
    for (auto& pixel: deadAndCo) {
//        std::cout << '(' << pixel[DEAD].y + 1 << "; " << pixel[DEAD].x + 1 << ')' << std::endl;

        for (int direction {TOP_LEFT}; direction <= BOTTOM_RIGHT; ++direction) {
            pixel[direction] = findNeighbour(pixel[DEAD], image, direction);
        }
    }



    // # Теперь заполняем выходной массив, согласно алгоритму
    std::vector<std::array<float, 14>> outputData (deadAndCo.size());
    formCorrectionData(deadAndCo, outputData);

    std::cout << "--------------------------------------------------------------------------------------------------"
              << "\nDead pixels map in the format: [X0, Y0, X1, Y1, W1, X2, Y2, W2, X3, Y3, W3, X4, Y4, W4], where is:"
              << "\nX0, Y0 - dead pixel column and row;"
              << "\nX1, Y1 - coordinates of the 1-st neighboring pixel involved in the correction,"
              << "\nW1 - weighting factor of that pixel;"
              << "\nX2..4, Y2..4, W2..4 - analogously."
              << "\n--------------------------------------------------------------------------------------------------"
              << std::endl;
    for (auto& data: outputData) {
        std::cout << '['
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[0] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[1] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[2] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[3] << ", "
                  << std::setw(5) << std::fixed << std::setprecision(2) << data[4] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[5] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[6] << ", "
                  << std::setw(5) << std::fixed << std::setprecision(2) << data[7] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[8] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[9] << ", "
                  << std::setw(5) << std::fixed << std::setprecision(2) << data[10] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[11] << ", "
                  << std::setw(2) << std::fixed << std::setprecision(0) << data[12] << ", "
                  << std::setw(5) << std::fixed << std::setprecision(2) << data[13] << ']'
                  << std::endl;
    }


//    for (auto& data: outputData) {

//        // # Считаем интенсивности соседей
//        float I1 = image.at<unsigned char>(data[Y1], data[X1]) * data[W1];
//        float I2 = image.at<unsigned char>(data[Y2], data[X2]) * data[W2];
//        float I3 = image.at<unsigned char>(data[Y3], data[X3]) * data[W3];
//        float I4 = image.at<unsigned char>(data[Y4], data[X4]) * data[W4];


//        // # Исправляем битый пиксель
//        image.at<unsigned char>(data[Y0], data[X0]) =   I1 + I2 + I3 + I4;

////        cv::namedWindow("Original", cv::WINDOW_KEEPRATIO);
////        cv::imshow("Original", image);
////        cv::resizeWindow("Original", 300, 200);
////        cv::waitKey(0);
//    }



    // # Создаём выходное изображение в виде карты ИСПРАВЛЕННЫХ пикселей
    cv::Mat pixelMap {cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC1)};
    for (auto& data: outputData) {

        // # Считаем интенсивности соседей
        float I1 = image.at<unsigned char>(data[Y1], data[X1]) * data[W1];
        float I2 = image.at<unsigned char>(data[Y2], data[X2]) * data[W2];
        float I3 = image.at<unsigned char>(data[Y3], data[X3]) * data[W3];
        float I4 = image.at<unsigned char>(data[Y4], data[X4]) * data[W4];


        // # Исправляем битый пиксель
        pixelMap.at<unsigned char>(data[Y0], data[X0]) =   I1 + I2 + I3 + I4;

        //        cv::namedWindow("Original", cv::WINDOW_KEEPRATIO);
        //        cv::imshow("Original", image);
        //        cv::resizeWindow("Original", 300, 200);
        //        cv::waitKey(0);
    }

    cv::namedWindow("PixelMap", cv::WINDOW_KEEPRATIO);
    cv::imshow("PixelMap", pixelMap);
    cv::resizeWindow("PixelMap", 300, 200);
    cv::waitKey(0);

    cv::imwrite(pixelMapPath.c_str(), pixelMap);

    std::cout << "\nDeadPixels finished its work correctly!" << std::endl;
    return 0;
}





//==================================================================================================
//          TYPE:   ........
//   DESCRIPTION:   Поиск ближайшего к @target небитого пикселя в указанном направлении
//    PARAMETERS:   ........
//  RETURN VALUE:   ........
// COMMENTS/BUGS:   ........
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
//          TYPE:   ........
//   DESCRIPTION:   Функция формирует список пикселей, по которым будет идти корректировка битого
//                  пикселя
//    PARAMETERS:   ........
//  RETURN VALUE:   ........
// COMMENTS/BUGS:   ........
//==================================================================================================
void formCorrectionData(std::vector<std::array<cv::Point2i, Direction::TOTAL>>& inputData, std::vector<std::array<float, 14>>& outputData)
{

    // # Нужно найти 4 ближайших точки, удовлетворяющих алгоритму
    for (int ii {0}; ii < inputData.size(); ++ii) {

        const std::array<cv::Point2i, Direction::TOTAL>& pixel {inputData[ii]};     // Битый пиксель с соседями

        // # Координаты битого пикселя
        outputData[ii][X0] = inputData[ii][DEAD].x;
        outputData[ii][Y0] = inputData[ii][DEAD].y;



        std::vector<cv::Point2i> correctionPoints {};                   // Вектор точек для коррекции текущего битого пикселя

        // # Сперва проверяем пиксели по горизонтали и вертикали
        const Line horizontal {pixel[LEFT], pixel[RIGHT]};
        const Line vertical {pixel[TOP], pixel[BOTTOM]};

        checkLine(horizontal, vertical, pixel[DEAD], MAX_DISTANCE, DISTANCE_DIFFERENCE_THRESHOLD, correctionPoints);

        // # Если достаточно горизонтальной и вертикальной линий - то заполняем соответствующими
        // # точками наш массив и переходим к следующему пикселю
        if (correctionPoints.size() > 0) {
            for (int kk {0}, id {2}; kk < correctionPoints.size(); ++kk, id += 2) {
                outputData[ii][id] = correctionPoints.at(kk).x;
                ++id;
                outputData[ii][id] = correctionPoints.at(kk).y;
            }

            // Заполнение весовых коэффициентов
            addFactors(outputData[ii], correctionPoints);
            continue;
        }
        else {
            // # Данных недостаточно - переходим к диагоналям
        }



        // # Теперь учитываем диагональные линии (с небольшой корректировкой MAX_DISTANCE)
        const Line forward {pixel[BOTTOM_LEFT], pixel[TOP_RIGHT]};
        const Line backward {pixel[TOP_LEFT], pixel[BOTTOM_RIGHT]};

        checkLine(forward, backward, pixel[DEAD], MAX_DISTANCE * sqrt(2.0) , DISTANCE_DIFFERENCE_THRESHOLD, correctionPoints);


        // # Если достаточно диагональных линий - то заполняем соответствующими точками наш массив
        // # и переходим к следующей точке
        if (correctionPoints.size() > 0) {
            for (int kk {0}, id {2}; kk < correctionPoints.size(); ++kk, id += 2) {
                outputData[ii][id] = correctionPoints.at(kk).x;
                ++id;
                outputData[ii][id] = correctionPoints.at(kk).y;
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
//          TYPE:   ........
//   DESCRIPTION:   Функция проверяет точки двух переданных линий на удовлетворение условиям
//                  алгоритма. Записывает от 0 до 4х точек в @retPoints
//    PARAMETERS:   ........
//  RETURN VALUE:   ........
// COMMENTS/BUGS:   ........
//==================================================================================================
void checkLine(const Line& lineAA, const Line& lineBB, const cv::Point2d& deadPoint, float maxDistance, float distDiffThreshold, std::vector<cv::Point2i>& retPoints)
{
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
            retPoints.push_back(lineAA.startPoint);
            retPoints.push_back(lineAA.endPoint);
        }
        else {}

        if (lineBBLength < distDiffThreshold) {
            retPoints.push_back(lineBB.startPoint);
            retPoints.push_back(lineBB.endPoint);
        }
        else {}


        // ## Обе линии НЕ удовлетворяют условию - используем только точки меньшей линии
        if (lineAALength >= distDiffThreshold && lineBBLength >= distDiffThreshold) {

            /***************************************************************************************
             * В случае, когда lineAALength == lineBBLength, просто использую одну из двух.
             **************************************************************************************/
            if (lineAALength <= lineBBLength) {
                retPoints.push_back(lineAA.startPoint);
                retPoints.push_back(lineAA.endPoint);
            }
            else {
                retPoints.push_back(lineBB.startPoint);
                retPoints.push_back(lineBB.endPoint);
            }
        }
        else {}
    }
    // # Только линия АА валидна
    else if (lineAAValid == true) {
        retPoints.push_back(lineAA.startPoint);
        retPoints.push_back(lineAA.endPoint);
    }
    // # Только линия BB валидна
    else if (lineBBValid == true) {
        retPoints.push_back(lineBB.startPoint);
        retPoints.push_back(lineBB.endPoint);
    }
    // # Никакая из линий не валидна
    else {}


}


//==================================================================================================
//          TYPE:   ........
//   DESCRIPTION:   Получение весовых коэффициентов точек линии.
//    PARAMETERS:   ........
//  RETURN VALUE:   ........
// COMMENTS/BUGS:   ........
//==================================================================================================
void addFactors(std::array<float, 14>& outputData, std::vector<cv::Point2i>& correctionPoints)
{
    float sum {0.0};
    const float WEIGHT_2 {1.0};
    const float WEIGHT_4 {0.5};

    // # Одна линия (используется 2 точки, вес линии 1.0)
    if (correctionPoints.size() == 2) {
        for (int ii {0}, w {W1}; ii < correctionPoints.size(); ++ii) {
            float length = std::sqrt(std::pow(correctionPoints.at(ii).x - outputData[X0], 2) + std::pow(correctionPoints.at(ii).y - outputData[Y0], 2));
            outputData[w] = length;
            w += 3;
            sum += 1.0 / length;
        }

        float temp {WEIGHT_2 / sum};
        for (int w {W1}; w <= W2; w += 3) {
            outputData[w] = temp / outputData[w];
        }
    }
    // # Две линии (используется 4 точки, вес каждой линии 0.5)
    else if (correctionPoints.size() == 4) {
        // ## Первая линия
        for (int ii {0}, w {W1}; ii < 2; ++ii) {
            float length = std::sqrt(std::pow(correctionPoints.at(ii).x - outputData[X0], 2) + std::pow(correctionPoints.at(ii).y - outputData[Y0], 2));
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
        for (int ii {2}, w {W3}; ii < correctionPoints.size(); ++ii) {
            float length = std::sqrt(std::pow(correctionPoints.at(ii).x - outputData[X0], 2) + std::pow(correctionPoints.at(ii).y - outputData[Y0], 2));
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
