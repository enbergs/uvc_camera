//
// Created by shaocheng on 9/8/18.
//
#include "circular_frame_info_array.h"

CircularFrameInfoArray::CircularFrameInfoArray(int max_size) {

    circular_array_.resize(max_size);
    max_size_ = max_size;
}

void CircularFrameInfoArray::push(const FrameInfo &frame_info) {
    circular_array_[curr_idx_] = frame_info;
    curr_idx_++;
    curr_idx_ %= max_size_;
}

FrameInfo& CircularFrameInfoArray::getNSteps(const int n) {
    int index = curr_idx_ - n;
    while (index < 0) {
        index += max_size_;
    }
    return circular_array_[index];


}

FrameInfo& CircularFrameInfoArray::getPrev() {
    return getNSteps(1);
}



