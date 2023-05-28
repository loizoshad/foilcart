'''
This script is used to generate a C++ header file from a given template.

The header file should be structured as follows:

    #pragma once
    #include <iostream.h>

    float a_{i}_{j}(std::vector<float> x, std::vector<float> u) 
    {
        return {function};
    }

    float b_{k}_{l}(std::vector<float> x, std::vector<float> u)
    {
        return {function};
    }

Then once all the aij, and bkl functions are written, then this script will generate the functions 
get_A() and get_B() for arbitrary NS and NC, and i, j, k, l as follows:


// Create a function that returns a Matrixxf of size (NS, NS)

Eigen::MatrixXf get_A() 
{
    std::map<std::string, std::function<int()>> functions = { {"a_1_1", a_1_1}, {"a_1_2", a_1_2}, {"a_2_1", a_2_1}, {"a_2_2", a_2_2} };

    Eigen::MatrixXf A(NS, NS);

    for (int i = 1; i <= NS; i++)
    {
        for (int j = 1; j <= NS; j++)
        {
            std::string function_name = "a" + std::to_string(i) + std::to_string(j);
            auto function = functions[function_name];
            A(i-1, j-1) = function();
        }
    }

    return A;
}

Eigen::MatrixXf get_B()
{
    std::map<std::string, std::function<int()>> functions = { {"b_1_1", b_1_1}, {"b_1_2", b_1_2}, {"b_2_1", b_2_1}, {"b_2_2", b_2_2} };

    Eigen::MatrixXf B(NS, NC);

    for (int i = 1; i <= NS; i++)
    {
        for (int j = 1; j <= NC; j++)
        {
            std::string function_name = "b" + std::to_string(i) + std::to_string(j);
            auto function = functions[function_name];
            B(i-1, j-1) = function();
        }
    }

    return B;
}
    

The header file will be created (if it doesn't exist) otherwise it will first delete it and create it again
by this script will have the name 'jacobian.h' and will be located in ../src/ relative
to the location of this script.

'''

import sys
import os
import re

from dynamic_model import DynamicModel
import casadi as ca
import casadi.tools as ct

def main():
    dynamic_model = DynamicModel()
    # functions = dynamic_model.compute_dynamics_jacobian()
    A, B = dynamic_model.compute_dynamics_jacobian()

    ns = dynamic_model.NS   # Number of states
    nc = dynamic_model.NC   # Number of controls

    # for i in range(0, ns*ns):
    #     print(f'A[{i}] = {A[i]}')
    # for i in range(0, ns*nc):
    #     print(f'B[{i}] = {B[i]}')

    if not os.path.exists(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h')):
        open(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h'), 'w').close()
    else:
        os.remove(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h'))
        open(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h'), 'w').close()    


    # Open the header file
    with open(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h'), 'a') as f:
        ##############################################################################
        #                                  Header                                    #
        ##############################################################################        
        # Write the header
        f.write('// This file is auto generated using the script \'generate_jacobian.py\'. Please do not edit. \r')
        f.write('#ifndef JACOBIAN_H \r')
        f.write('#define JACOBIAN_H \r')
        # f.write('#include <vector> \r')
        # f.write('#include <functional> \r')
        # f.write('#include <map> \r')
        f.write('#include <ArduinoEigen.h> \r')
        f.write('#include <Arduino.h> \r \r')

        f.write(f'#define NS {ns} \r')
        f.write(f'#define NC {nc} \r \r')

        ##############################################################################
        #                           a_{i}_{j}, b_{k}_{l}                             #
        ##############################################################################
        for i in range(1, ns+1):
            for j in range(1, ns+1):
                f.write(f'inline float a_{i}_{j}(std::vector<float> x, std::vector<float> u) \r')
                f.write('{ \r')
                f.write(f'    return {A[(i-1)*ns + j-1]}; \r')
                f.write('} \r \r')

            for j in range(1, nc+1):
                f.write(f'inline float b_{i}_{j}(std::vector<float> x, std::vector<float> u) \r')
                f.write('{ \r')
                f.write(f'    return {B[(i-1)*nc + j-1]}; \r')
                f.write('} \r \r')

        ##############################################################################
        #                                 get_A()                                    #
        ##############################################################################
        f.write('// Create a function that returns a Matrixxf of size (NS, NS) \r \r')
        f.write('inline Eigen::MatrixXf get_A(std::vector<float> x, std::vector<float> u) \r')
        f.write('{ \r')
        f.write('    std::map<std::string, std::function<float(std::vector<float>, std::vector<float>)>> functions = { \r')

        for i in range(1, ns + 1):
            for j in range(1, ns + 1):
                if i == ns and j == ns:
                    f.write(f'        {{ "a_{i}_{j}", a_{i}_{j} }} \r')
                else:
                    f.write(f'        {{ "a_{i}_{j}", a_{i}_{j} }}, ')
        f.write('    }; \r \r')

        f.write('    Eigen::MatrixXf A(NS, NS); \r \r')

        f.write('    for (int i = 1; i <= NS; i++) \r')
        f.write('    { \r')
        f.write('        for (int j = 1; j <= NS; j++) \r')
        f.write('        { \r')
        f.write('            String function_name = {"a_" + String(i) + "_" + String(j)}; \r')
        f.write('            std::string function_name_std = function_name.c_str(); \r')
        f.write('            auto function = functions[function_name_std]; \r')
        f.write('            A(i-1, j-1) = function(x, u); \r')
        f.write('        } \r')
        f.write('    } \r \r')

        f.write('    return A; \r')
        f.write('} \r \r')

        
        ##############################################################################
        #                                 get_B()                                    #
        ##############################################################################
        f.write('inline Eigen::MatrixXf get_B(std::vector<float> x, std::vector<float> u) \r')
        f.write('{ \r')
        f.write('    std::map<std::string, std::function<float(std::vector<float>, std::vector<float>)>> functions = { \r')
        
        for i in range(1, ns + 1):
            for j in range(1, nc + 1):
                if i == ns and j == nc:
                    f.write(f'        {{ "b_{i}_{j}", b_{i}_{j} }} \r')
                else:
                    f.write(f'        {{ "b_{i}_{j}", b_{i}_{j} }}, ')
        f.write('    }; \r \r')

        f.write('    Eigen::MatrixXf B(NS, NC); \r \r')

        f.write('    for (int i = 1; i <= NS; i++) \r')
        f.write('    { \r')
        f.write('        for (int j = 1; j <= NC; j++) \r')
        f.write('        { \r')
        f.write('            String function_name = {"b_" + String(i) + "_" + String(j)}; \r')
        f.write('            std::string function_name_std = function_name.c_str(); \r')
        f.write('            auto function = functions[function_name_std]; \r')
        f.write('            B(i-1, j-1) = function(x, u); \r')
        f.write('        } \r')
        f.write('    } \r \r')

        f.write('    return B; \r')
        f.write('} \r \r')

        ##############################################################################
        #                                  Footer                                    #
        ##############################################################################
        f.write('#endif \r')


if __name__ == '__main__':
    main()


