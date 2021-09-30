//
// Created by ubuntu on 7/14/21.
//

#ifndef PCD_TEST_KDTREE3D_H
#define PCD_TEST_KDTREE3D_H

#include <pcl/impl/point_types.hpp>
#include <vector>

struct Node {
    pcl::PointXYZ point;
    int id;
    Node *left;
    Node *right;

    Node(pcl::PointXYZ arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
    Node *root;

    KdTree() : root(NULL) {}

    void insertHelper(Node **node, int depth, pcl::PointXYZ point, int id) {
        // Tree is empty
        if (*node == NULL) {
            *node = new Node{point, id};
        } else {
            // calculate current din
            int cd = depth % 3;
            if (cd == 0) {
                if (point.x < (*node)->point.x) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            } else if (cd == 1) {
                if (point.y < (*node)->point.y) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            } else {
                if (point.z < (*node)->point.z) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            }
        }
    }

    void insert(pcl::PointXYZ point, int id) {
        // the function should create a new node and place correctly with in the root
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(pcl::PointXYZ target, Node *node, int depth, float distanceTolx, float distanceToly, float distanceTolz, std::vector<int> &ids) {
        if (node != NULL) {
            float delta_x = node->point.x - target.x;
            float delta_y = node->point.y - target.y;
            float delta_z = node->point.z - target.z;

            float thresh = 0.02*target.z + 0.8;
//            if(target.z > 120)
//                thresh = 2.7;

            if ((delta_x >= -distanceTolx && delta_x <= distanceTolx) &&
                (delta_y >= -distanceToly && delta_y <= distanceToly) &&
                (delta_z >= -distanceTolz && delta_z <= distanceTolz)) {
                float distance = sqrt(delta_x * delta_x/(distanceTolx*distanceTolx) + delta_y * delta_y/(distanceToly*distanceToly) + delta_z * delta_z/(distanceTolz*distanceTolz));
                if (distance <= thresh) {
                    ids.push_back(node->id);
                }
            }

//            if ((delta_x >= -distanceTolx && delta_x <= distanceTolx) &&
//                (delta_y >= -distanceToly && delta_y <= distanceToly) &&
//                (delta_z >= -distanceTolz && delta_z <= distanceTolz)) {
//                float distance = sqrt(delta_x * delta_x/0.04 + delta_y * delta_y/4.0 + delta_z * delta_z/(1.6*1.6));
//                if (distance <= thresh) {
//                    ids.push_back(node->id);
//                }
//            }
            // check across boundary
            if (depth % 3 == 0) {
                if (delta_x > -distanceTolx) {
                    searchHelper(target, node->left, depth + 1, distanceTolx, distanceToly, distanceTolz, ids);
                }
                if (delta_x < distanceTolx) {
                    searchHelper(target, node->right, depth + 1, distanceTolx, distanceToly, distanceTolz, ids);
                }
            } else if (depth % 3 == 1) {
                if (delta_y > -distanceToly) {
                    searchHelper(target, node->left, depth + 1, distanceTolx, distanceToly, distanceTolz, ids);
                }
                if (delta_y < distanceToly) {
                    searchHelper(target, node->right, depth + 1, distanceTolx, distanceToly, distanceTolz, ids);
                }
            } else {
                if (delta_z > -distanceTolz) {
                    searchHelper(target, node->left, depth + 1, distanceTolx, distanceToly, distanceTolz, ids);
                }
                if (delta_z < distanceTolz) {
                    searchHelper(target, node->right, depth + 1, distanceTolx, distanceToly, distanceTolz, ids);
                }
            }

        }
    }
    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(pcl::PointXYZ target, float distanceTolx, float distanceToly, float distanceTolz)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTolx, distanceToly, distanceTolz, ids);
        return ids;
    }

};

#endif //PCD_TEST_KDTREE3D_H
