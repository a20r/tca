
#ifndef WEIGHT_H
#define WEIGHT_H

template <class T>
class Weight {

    public:
        Weight() {};

        Weight(T val, double weight) {
            this->val = val;
            this->weight = weight;
        }

        T get_val() const {
            return this->val;
        }

        double get_weight() const {
            return this->weight;
        }

        bool operator<(const Weight& right) const {
            return this->weight < right.get_weight();
        }

    private:
        T val;
        double weight;

};

#endif
