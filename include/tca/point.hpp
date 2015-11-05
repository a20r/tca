
#ifndef TCA_POINT_H
#define TCA_POINT_H

class Point {

    public:
        Point(double x, double y) : x(x), y(y) {}

        double get_x() {
            return this->x;
        }

        double get_y() {
            return this->y;
        }

        void set_x(double x) {
            this->x = x;
        }

        void set_y(double y) {
            this->y = y;
        }

    private:
        double x, y;
};
