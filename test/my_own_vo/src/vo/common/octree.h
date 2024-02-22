/**
 * Copyright (C), 2023
 * @file 	 octree.h
 * @author 	 peitianyu(https://github.com/peitianyu)
 * @date 	 2023-09-28
 * @brief 	 [简介] 实现一个四叉树/八叉树类
 */
#ifndef __OCTREE_H__
#define __OCTREE_H__

#include <vector>
#include <functional>

template <typename PosType, typename DataType, size_t DIM>
class Octree {
public:
    constexpr static size_t child_num_ = 1 << DIM;
    struct Node
    {
        PosType center;
        DataType data;
        size_t depth;
        Node *childs[child_num_];

        Node() : center(PosType()), data(DataType()), depth(0) {
            for (size_t i = 0; i < child_num_; ++i) childs[i] = nullptr;
        }

        Node(const PosType& center, const DataType& data, size_t depth) 
            : center(center), data(data), depth(depth) {
            for (size_t i = 0; i < child_num_; ++i) childs[i] = nullptr;
        }

        ~Node() { 
            for (size_t i = 0; i < child_num_; ++i) {
                if (childs[i] != nullptr) {
                    delete childs[i];
                }
            }
        }
    };

    struct Boundary
    {
        PosType min;
        PosType max;

        Boundary(PosType min, PosType max) : min(min), max(max) { }
        Boundary() : min(PosType::Zero()), max(PosType::Zero()) { }

        /**
         * @brief 	 [简介] 获取边界尺寸
         * @return 	 [constexpr PosType] 返回边界尺寸
         */
        constexpr PosType size() const { return max - min; }

        /**
         * @brief 	 [简介] 获取边界中心
         * @return 	 [constexpr PosType] 返回边界中心位置
         */
        constexpr PosType center() const { return (max + min) / 2; }

        /**
         * @brief 	 [简介] 判断点是否在边界内
         * @param 	 pos [in], 点位置 
         * @return 	 [true] or [false]
         */
        bool is_in(const PosType& pos)
        {
            for (size_t i = 0; i < DIM; ++i) {
                if (pos[i] < min[i] || pos[i] > max[i]) return false;
            }
            return true;
        }
    };

    /**
     * @brief 	 [简介] 构造函数
     * @param 	 min [in], 边界最小值 
     * @param 	 max [in], 边界最大值
     * @param 	 depth [in], 最大深度
     */
    Octree(const PosType& min, const PosType& max, size_t depth) 
        :boundary_(Boundary(min, max)), max_depth_(depth)
    {
        root_ = new Node(boundary_.center(), DataType(), 0);
    }

    /**
     * @brief 	 [简介] 析构函数
     */
    ~Octree()
    {   
        delete root_;
    }

    /**
     * @brief 	 [简介] reset函数
     */
    void reset()
    {
        delete root_;
        root_ = new Node(boundary_.center(), DataType(), 0);
    }
    

    /**
     * @brief 	 [简介] 用于插入点
     * @param 	 pos [in], 点位置 
     * @param 	 data [in], 所带数据
     */
    void insert(const PosType& pos, const DataType& data)
    {
        // TODO: 之后加点越界打印
        if(!boundary_.is_in(pos)) return;

        insert(root_, pos, data);
    }

    /**
     * @brief 	 [简介] 用于查找点
     * @param 	 pos [in], 点位置 
     * @param 	 depth [in], 查找深度
     * @return 	 [Node*], 返回节点指针
     */
    Node *find(const PosType& pos, const int& depth = -1) { 
        return depth == -1 ? find(root_, pos, max_depth_) : find(root_, pos, depth); 
    }

    /**
     * @brief 	 [简介] 找到节点的边界
     * @param 	 node [in], 需要找到边界的节点 
     * @param 	 boundary [in], 边界 
     */
    void find_boundary(Node* node, Boundary& boundary)
    {
        PosType half_size = boundary_.size() / (1 << (node->depth + 1));
        boundary.min = node->center - half_size;
        boundary.max = node->center + half_size;
    }

    /**
     * @brief 	 [简介] 删除节点
     * @param    pos [in], 点位置
     * @param    depth [in], 删除深度
     */
    void remove(const PosType& pos, const size_t& depth)
    {
        if(depth == 0) reset();

        remove(root_, pos, depth);
    }

    /**
     * @brief 	 [简介] 可视化树
     * @param 	 func [in], 可视化函数 
     */
    void visual(std::function<void(Node* node)> func = nullptr) { traverse(root_, func);}
protected:
    /**
     * @brief 	 [简介] 遍历树
     * @param 	 node [in], 需要遍历的节点 
     * @param 	 func [in], 遍历函数
     */
    void traverse(Node *node, std::function<void(Node* node)> func)
    {
        if (node == nullptr) return;
        func(node);
        for (size_t i = 0; i < child_num_; i++) {
            if(node->childs[i] != nullptr){
                traverse(node->childs[i], func);
            }
        }
    }

    /**
     * @brief 	 [简介] 插入点
     * @param 	 node [in], 插入节点
     * @param 	 pos [in], 插入点位置
     * @param 	 data [in], 插入点数据
     * @note 	 [注意] 递归插入
     */
    void insert(Node *node, const PosType& pos, const DataType& data)
    {
        if (node->depth+1 == max_depth_) return;

        size_t index = find_index(pos, node);

        if (node->childs[index] == nullptr) {
            node->childs[index] = new Node(find_center(pos, node), data, node->depth + 1);
        }else{
            node->childs[index]->data = update(node->childs[index]->data, data);
        }

        insert(node->childs[index], pos, data);
    }
    
    /**
     * @brief 	 [简介] 更新节点数据方法, 默认为加法
     * @param 	 old_data [in], 原节点数据 
     * @param 	 new_data [in], 新节点数据
     * @return 	 [DataType] 返回更新后的数据
     */
    virtual DataType update(DataType& old_data, const DataType& new_data)
    {
        return (new_data + old_data);
    }

    /**
     * @brief 	 [简介] 查找节点
     * @param 	 node [in], 查找节点 
     * @param 	 pos [in], 查找点位置 
     * @param 	 depth [in], 查找深度 
     * @return 	 [Node*] 返回查找到的节点
     * @note 	 [注意] 递归查找
     */
    Node *find(Node *node, const PosType& pos, const size_t& depth)
    {
        if (node->depth == depth) return node;

        size_t index = find_index(pos, node);

        if (node->childs[index] == nullptr) return node;

        return find(node->childs[index], pos, depth);
    }

    /**
     * @brief 	 [简介] erase方法
     * @param 	 node [in], 删除节点
     * @param 	 pos [in], 删除点位置
     * @param 	 depth [in], 删除深度
     */
    void remove(Node *node, const PosType& pos, const size_t& depth)
    {
        size_t index = find_index(pos, node);
        if (node->childs[index] == nullptr)     return;

        if (node->childs[index]->depth == depth) {
            delete node->childs[index];
            node->childs[index] = nullptr;
            return;
        }
        remove(node->childs[index], pos, depth);
    }
private:
    /**
     * @brief 	 [简介] 找到点所在的区域
     * @param 	 pos [in], 点位置
     * @param 	 node [in], 所在节点
     * @return 	 [size_t] 返回点所在区域的id
    */
    size_t find_index(const PosType& pos, const Node *node)
    {
        size_t index = 0;
        for (size_t i = 0; i < DIM; ++i) {
            if (pos[i] > node->center[i]) index |= (1 << i);
        }
        return index;
    }

    /**
     * @brief 	 [简介] 找到点所在区域的中心
     * @param 	 pos [in], 点位置 
     * @param 	 node [in], 所在节点
     * @return 	 [PosType] 返回点所在区域的中心
     */
    PosType find_center(const PosType& pos, const Node *node)
    {
        PosType center = node->center;
        PosType half_size = boundary_.size() / (1 << (node->depth + 2));
        for (size_t i = 0; i < DIM; ++i) {
            center[i] = (pos[i] > node->center[i]) ? center[i] + half_size[i] : center[i] - half_size[i];
        }

        return center;
    }
private:
    Boundary boundary_;
    size_t max_depth_;
    Node *root_;
};

template<typename PosType, typename DataType> using QuadTree = Octree<PosType, DataType, 2>;
template<typename PosType, typename DataType> using OctTree = Octree<PosType, DataType, 3>;

#endif // __OCTREE_H__