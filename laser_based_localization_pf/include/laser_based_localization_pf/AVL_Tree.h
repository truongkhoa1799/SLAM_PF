#include <iostream>
#include "Eigen/Dense"
using namespace std;

class Node
{
public:
    Node* left;
    Node* right;

    int value;
    int height;
    int mu[2];
    Eigen::MatrixXd cov;

    Node(int value, int _mu[2] , Eigen::MatrixXd _cov)
    {
        this->value = value;
        this->mu[0] = _mu[0];
        this->mu[1] = _mu[1];
        this->cov = _cov;
        left = right = nullptr;
        height = 1;
    }
};
class AVL
{
private:
    Node* head;
    int getHeight(Node* node)
    {
        if(node == nullptr)
            return 0;

        return node->height;
    }
    Node* updateHeight(Node* node)
    {
        node->height = 1 + max(getHeight(node->left), getHeight(node->right));
        return node;
    }
    
    Node* leftRotate(Node* root)
    {
        Node* x = root->right;
        Node* t = x->left;

        root->right = t;
        x->left = root;

        // update height
        x = updateHeight(x);
        root = updateHeight(root);

        return x;
    }
    Node* rightRotate(Node* root)
    {
        Node *x = root->left;
        Node *t = x->right;

        root->left = t;
        x->right = root;

        // update height
        root = updateHeight(root);
        x = updateHeight(x);

        return x;
    }
    
    Node* insert(Node* root, int value, int _mu[2] , Eigen::MatrixXd _cov)
    {
        if(root == nullptr)
            return new Node(value , _mu , _cov);

        if(root->value < value) root->right = insert(root->right, value , _mu , _cov);
        else if(root->value == value)
        {
            cout<<"value: "<<value<<" No duplicate vertex allowed."<<endl;
            return root;
        }
        else
            root->left = insert(root->left, value , _mu , _cov);

        root = updateHeight(root);

        int balance_factor = getHeight(root->left) - getHeight(root->right);

        // LR rotation
        if(balance_factor > 1 && root->left->value < value)
        {
            root->left = leftRotate(root->left);
            return rightRotate(root);
        }
        // RR rotation
        else if(balance_factor > 1 && root->left->value > value) return rightRotate(root);
        // LL rotation
        else if(balance_factor < -1 && root->right->value < value) return leftRotate(root);
        // RL rotation
        else if(balance_factor < -1 && root->right->value > value)
        {
            root->right = rightRotate(root->right);
            return leftRotate(root);
        }

        return root;
    }
    void print_recur(Node* root)
    {
        if(root->left != nullptr)
            print_recur(root->left);

        cout<<root->value<<" ";

        if(root->right != nullptr)
            print_recur(root->right);
    }
public:
    AVL()
    {
        this->head = NULL;
    }
    void insert_Node(int value, int _mu[2] , Eigen::MatrixXd _cov)
    {
        this->head = insert(this->head, value , _mu , _cov);
    }
    void printAll()
    {
        print_recur(this->head);
    }
};
