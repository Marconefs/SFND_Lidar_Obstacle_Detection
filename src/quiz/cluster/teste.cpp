#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>


void teste(int **a,int *b, int c);

void teste(int **a, int &b, int c)
{
    std::cout << "\n  a:" << a; // imprime o endereco
    std::cout << "\n *a:" << *a; // imprime outro endereco
    std::cout << "\n**a:" << **a; // imprime o valor
    std::cout << "\n  b:" << b; // imprime o valor
    std::cout << "\n  c:" << c; // imprime o valor
    std::cout << "\n";
    **a = 4;
    b = 3;
    c = 5;
}

int main ()
{
    int *a = new int();
    *a = 2;
    int b = 1;
    int c = 0;
    std::cout << "\n a:" << a;  // imprime o endereco
    std::cout << "\n *a:" << *a; // imprime o valor
    teste(&a, b, c);
    std::cout << "\n\n b:" << b;  // imprime o valor 3
    std::cout << "\n  *a:" << *a; // imprime o valo 4
    std::cout << "\n   c:" << c; // imprime o valo 0
    std::cout << "\n";
}