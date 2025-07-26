#include <iostream>
#include <opencv2/opencv.hpp>


#define     MAX_DISTANCE                    2       // Расстояние от битого пикселя до ближайшего исправного (в любом направлении, просто: X ---> Y )
#define     DISTANCE_DIFFERENCE_THRESHOLD   6       // Расстояние между исправными пикселями, т.е.: Y1 <--- X ---> Y2;  X - битый пиксель, Y - исправный
#define     INTENCITY_LEVEL                 128



// # Структура описывает координаты битого пикселя и его ближайших исправных соседей
class Neighbors {
public:

    // # Мертвый пиксель
    cv::Point2i dead {};

    // # Горизонталь
    cv::Point2i left {};
    cv::Point2i right {};

    // # Вертикаль
    cv::Point2i top {};
    cv::Point2i bottom {};

    // # Диагональ 1
    cv::Point2i topLeft {};
    cv::Point2i bottomRight {};

    // # Диагональ 2
    cv::Point2i bottomLeft {};
    cv::Point2i topRight {};

    Neighbors(int x, int y) :
        dead {x, y},
        left {x, y},
        right {x, y},
        top {x, y},
        bottom {x, y},
        topLeft {x, y},
        bottomRight {x, y},
        bottomLeft {x, y},
        topRight {x, y}
    {

    }
};


int main()
{
    const std::string imagePath {"/home/alexey/Programming/Projects/Qt/DeadPixel/Task/test.png"};
    std::vector<Neighbors> deadAndCo {};

    cv::Mat image {cv::imread(imagePath, cv::IMREAD_GRAYSCALE)};
    if (image.isContinuous() == false) {
        image = image.clone();
    }


    cv::namedWindow("Original", cv::WINDOW_KEEPRATIO);
    cv::imshow("Original", image);
    cv::resizeWindow("Original", 300, 200);
    cv::waitKey(0);


    const int imageWidth {image.cols};
    const int imageHeight {image.rows};


    // # Ищем битые пиксели
    for (int row {0}; row < imageHeight; ++row) {
        for (int column {0}; column < imageWidth; column++) {
            if (image.at<unsigned char>(row, column) >= INTENCITY_LEVEL) {
                deadAndCo.push_back({column, row});
            }
        }
    }


    // # Выводим список битых пикселей в формате (столбец, строка)
    for (const auto& pixel: deadAndCo) {
        std::cout << '(' << pixel.dead.x + 1 << "; " << pixel.dead.y + 1 << ")\n";
    }
    std::cout << std::endl;


    // # Теперь ищем ближайших соседей по горизонтали и вертикали
    for (auto& pixel: deadAndCo) {

        const int deadColumn {pixel.dead.x};
        const int deadRow {pixel.dead.y};

        // # Слева
        for (int column {deadColumn}; column >= 0; --column) {
            if (image.at<unsigned char>(deadRow, column) < INTENCITY_LEVEL) {
                pixel.left.x = column;
                break;
            }
            else {}
        }

        // # Справа
        for (int column {deadColumn}; column < imageWidth; ++column) {
            if (image.at<unsigned char>(deadRow, column) < INTENCITY_LEVEL) {
                pixel.right.x = column;
                break;
            }
            else {}
        }


        // # Сверху
        for (int row {deadRow}; row >= 0; --row) {
            if (image.at<unsigned char>(row, deadColumn) < INTENCITY_LEVEL) {
                pixel.top.y = row;
                break;
            }
            else {}
        }

        // # Снизу
        for (int row {deadRow}; row < imageHeight; ++row) {
            if (image.at<unsigned char>(row, deadColumn) < INTENCITY_LEVEL) {
                pixel.bottom.y = row;
                break;
            }
            else {}
        }



        // # СлеваСверху
        for (int column {deadColumn}, row {deadRow}; column >= 0, row >= 0; --column, --row) {
            if (image.at<unsigned char>(row, column) < INTENCITY_LEVEL) {
                pixel.topLeft.x = column;
                pixel.topLeft.y = row;
                break;
            }
            else {}
        }

        // # СнизуСправа
        for (int column {deadColumn}, row {deadRow}; column < imageWidth, row < imageHeight; ++column, ++row) {
            if (image.at<unsigned char>(row, column) < INTENCITY_LEVEL) {
                pixel.bottomRight.x = column;
                pixel.bottomRight.y = row;
                break;
            }
            else {}
        }



        // # СнизуСлева
        for (int column {deadColumn}, row {deadRow}; column >= 0, row < imageHeight; --column, ++row) {
            if (image.at<unsigned char>(row, column) < INTENCITY_LEVEL) {
                pixel.bottomLeft.x = column;
                pixel.bottomLeft.y = row;
                break;
            }
            else {}
        }


        // # СверхуСправа
        for (int column {deadColumn}, row {deadRow}; column < imageWidth, row >= 0; ++column, --row) {
            if (image.at<unsigned char>(row, column) < INTENCITY_LEVEL) {
                pixel.topRight.x = column;
                pixel.topRight.y = row;
                break;
            }
            else {}
        }


    }







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
















