#include "AVL_Tree.h"
int main()
{
    AVL *avl = new AVL();
    int mu[2];
    Eigen::MatrixXd cov(2,2);
    mu[0] = 1.0;
    mu[1] = 1.0;
    cov << 1.0 , 0 , 0 , 1.0;
    avl->insert_Node(10 , mu , cov);
    avl->insert_Node(1 , mu , cov);
    avl->insert_Node(30 , mu , cov);
    avl->insert_Node(40, mu , cov);
    avl->insert_Node(50, mu , cov);
    avl->insert_Node(25, mu , cov);
    avl->insert_Node(3, mu , cov);

    avl->printAll();

    return 0;
}

