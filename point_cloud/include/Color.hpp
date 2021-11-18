//
//  Color.hpp
//  PCL
//
//  Created by Jaromír Landa on 21/10/2021.
//

#ifndef Color_hpp
#define Color_hpp

#include <stdio.h>

struct Color {

    int R;
    int G;
    int B;
    int A;

    Color(int r, int g, int b, int a){
        this->R = r;
        this->G = g;
        this->B = b;
        this->A = a;
    }
    
    Color(int r, int g, int b){
        this->R = r;
        this->G = g;
        this->B = b;
        this->A = 0;
    }

};

#endif /* Color_hpp */
