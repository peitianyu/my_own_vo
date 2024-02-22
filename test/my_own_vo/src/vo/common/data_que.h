#ifndef __DATA_QUE_H__
#define __DATA_QUE_H__

#include <vector>

template <typename T>
class DataQue
{
public:
    DataQue(const uint& max_size = 4)
        : max_size_(max_size) 
    {
        data_.reserve(max_size_);
    }

    void push(const T& data)
    {
        if (data_.size() < max_size_) {
            data_.push_back(data);
        } else {
            data_.erase(data_.begin());
            data_.push_back(data);
        }
    }

    T pop()
    {
        T data = data_.front();
        data_.erase(data_.begin());
        return data;
    }

    T& operator[](const uint& idx) { return data_[idx]; } // 不建议使用

    uint size() const { return data_.size(); }

    bool empty() const { return data_.empty(); }

    void clear() { data_.clear(); }

    T& front() { return data_.front(); }

    T& back() { return data_.back(); }

    std::vector<T>& data() { return data_; }

    const std::vector<T>& data() const { return data_; }
private:
    std::vector<T> data_;
    uint max_size_;
};

#endif // __DATA_QUE_H__