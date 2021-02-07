#include "../header/driver.h"
#include <iostream>

int main(int argc, char** argv)
{
    Points tgt("pc1Error.pcd", "SIFT");
    Points src("pc2Error.pcd","SIFT");

    CloudRegistration registrar;
    registrar.Register(src, tgt, "pipeline1");

    return 0;
}
