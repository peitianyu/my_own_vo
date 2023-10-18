#ifndef __INI_PARSE_H__
#define __INI_PARSE_H__

#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>
#include<map>
#include<cstdlib>
#include<Eigen/Core>
#include<assert.h>


struct IniNode
{
	IniNode(std::string _root, std::string _key, std::string _value)
	:root(_root), key(_key), value(_value)
	{}

	std::string root;
	std::string key; 
	std::string value;
};

struct SubNode
{
	void insert_element(std::string key, std::string value) {
		sub_node.insert(std::pair<std::string, std::string>(key, value));
	}
	std::map<std::string, std::string> sub_node;
};

class IniParse
{
public:
    IniParse(const std::string& path);

    template<typename T>
    T get_val(std::string root, std::string key)
    {
        std::string str = get_str(root, key);
        assert(str != m_non_value_return && "get_val failed");

        std::istringstream iss(str);
        T val;
        iss >> val;
        return val;
    }

    template<typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> get_vec(std::string root, std::string key)
    {
        std::vector<T> vec_tmp = get_vector<T>(root, key);

        Eigen::Matrix<T, Eigen::Dynamic, 1> vec(vec_tmp.size());
        for(uint i = 0; i < vec_tmp.size(); ++i) vec(i) = vec_tmp[i];
        return vec;
    }

    template<typename T>
    Eigen::Matrix3d get_mat(std::string root, std::string key)
    {
        std::vector<T> vec_tmp = get_vector<T>(root, key);

        Eigen::Matrix3d mat;
        for(uint i = 0; i < 3; ++i)
        for(uint j = 0; j < 3; ++j)
            mat(i, j) = vec_tmp[i * 3 + j];
        return mat;
    }

	void clear();
private:
    void read_ini(std::string path);

    std::string get_str(std::string root, std::string key);

    template<typename T>
    std::vector<T> get_vector(std::string root, std::string key)
    {
        std::string str = get_str(root, key);
        assert(str != m_non_value_return && "get_val failed");
        
        std::replace(str.begin(), str.end(), ',', ' ');
        std::istringstream iss(str);
        std::vector<T> vec;
        T tmp;
        while(iss >> tmp) vec.push_back(tmp);
        return vec;
    }

	std::string& trim_str(std::string& str);
private:
	std::map<std::string, SubNode> m_map_ini;

	const std::string m_non_value_return = "NONE_VALUE_IN_CONFIG_FILE";
};



#endif // __INI_PARSE_H__