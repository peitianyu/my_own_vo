#ifndef __TYPES_DATAQUE_H__
#define __TYPES_DATAQUE_H__

#include <vector>

template<class DataType, std::size_t N>
class DataQue
{
public:
    DataQue() { m_data.reserve(N); }

    DataQue(const DataQue& other) : m_data(other.m_data) {}

    void PushBack(const DataType& data)
    {
        m_data.push_back(data);
        if(m_data.size() > N)
            m_data.erase(m_data.begin());
    }

    DataType PopFront()
    {
        DataType ret = m_data.front();
        m_data.erase(m_data.begin());
        return ret;
    }

    DataType PopBack()
    {
        DataType ret = m_data.back();
        m_data.pop_back();
        return ret;
    }

    const DataType& Front() const { return m_data.front();}

    const DataType& Back() const { return m_data.back();}

    size_t Size() { return m_data.size(); }

    size_t Empty() { return m_data.empty(); }

    void Reset() { return m_data.clear(); }

    const std::vector<DataType>& Data() { return m_data; }
protected:
    std::vector<DataType> m_data;
};

#endif // __TYPES_DATAQUE_H__