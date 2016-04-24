/*
 * Copyright 2016 <Lennart Nachtigall> <firesurfer65@yahoo.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


#pragma once


#include <iostream>
#include <vector>
#include <string>
#include <type_traits>
#include "rclcpp/rclcpp.hpp"

#define ASTRINGZ(x) STRINGZ(x)
#define STRINGZ(x)  #x
#define REFLECT(a)  addElement(ASTRINGZ(a) , a);
#define UNUSED(x) (void)(x)


using namespace std;

class Element{
public:
    string key;
    
    
    virtual void print(){}
    virtual rclcpp::parameter::ParameterVariant  getParameterVariant()=0;
    virtual rclcpp::parameter::ParameterVariant  getParameterVariant(std::string prefix)= 0;
    
    
};
template <class T>
class SpecificElement : public Element{
    
    static_assert(std::is_same<T, double>::value || std::is_same<T, int64_t>::value || std::is_same<T, bool>::value || std::is_same<T, std::string>::value || std::is_same<T, std::vector<uint8_t>>::value,
                  "The Reflection api only accepts the following types: double, int64_t, boo, string, std::vector<uint8_t>");
public: 
    T &data;
    SpecificElement(string _key,T &_data):data(_data){
        key=_key;
    }
    virtual void print(){
        cout<<"<"<<key<<"> ="<<data<<endl;
    }
    virtual void setValue(T val)
    {
        data =val;
    }
    virtual rclcpp::parameter::ParameterVariant getParameterVariant()
    {
        return rclcpp::parameter::ParameterVariant(key, data);
    }
    virtual rclcpp::parameter::ParameterVariant getParameterVariant(std::string prefix)
    {
        return rclcpp::parameter::ParameterVariant(prefix+"."+key, data);
    }
};
