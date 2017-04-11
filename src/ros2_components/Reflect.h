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
#include <cstring>

#define ASTRINGZ(x) STRINGZ(x)
#define STRINGZ(x)  #x
#define REFLECT(a)  addElement(ASTRINGZ(a) , a);
#define UNUSED(x) (void)(x)


using namespace std;

class Element{
public:
    string key;
    string type;

    virtual std::string getKey(){return key;}
    virtual std::string getType(){return type;}
    virtual size_t getSize()=0;
    virtual void print(){}
    virtual rclcpp::parameter::ParameterVariant  getParameterVariant()=0;
    virtual rclcpp::parameter::ParameterVariant  getParameterVariant(std::string prefix)= 0;
    virtual const void * getVoidPtr(uint8_t & length, std::string & _key, std::string & _type)=0;
    virtual std::vector<uint8_t> getBytes(std::string & _key, std::string &_type)=0;
    
};
template <class T>
class SpecificElement : public Element{
    
    static_assert(std::is_same<T, double>::value || std::is_same<T, int64_t>::value || std::is_same<T, bool>::value || std::is_same<T, std::string>::value || std::is_same<T, std::vector<uint8_t>>::value,
                  "The Reflection api only accepts the following types: double, int64_t, boo, string, std::vector<uint8_t>");
public: 
    T &data;
    SpecificElement(string _key,T &_data):data(_data){
        key=_key;
        if(std::is_same<T, double>::value)
            type = "double";
        else if(std::is_same<T, int64_t>::value)
            type = "int64_t";
        else if(std::is_same<T, bool>::value)
            type = "bool";
        else if(std::is_same<T, std::vector<uint8_t>>::value)
            type = "vector<uint8_t>";
    }
    virtual void print(){
        cout<<"<"<<key<<"> ="<<data<<endl;
    }
    virtual void setValue(T val)
    {
        data =val;
    }
    virtual size_t getSize()
    {
        return sizeof(T);
    }

    virtual rclcpp::parameter::ParameterVariant getParameterVariant()
    {
        return rclcpp::parameter::ParameterVariant(key, data);
    }
    virtual rclcpp::parameter::ParameterVariant getParameterVariant(std::string prefix)
    {
        return rclcpp::parameter::ParameterVariant(prefix+"."+key, data);
    }
    virtual const void * getVoidPtr(uint8_t & length, std::string & _key,std::string &_type)
    {
        _key = key;
        _type = type;
        length = sizeof(T);
        return &data;
    }
    virtual std::vector<uint8_t> getBytes(std::string & _key, std::string &_type)
    {
        _key = key;
        _type = type;
        size_t length = sizeof(T);
        std::vector<uint8_t> bytes;
        bytes.reserve(length);
        bytes.resize(length);
        std::memcpy(bytes.data(),&data,length);
        return bytes;
    }
};

template <>
class SpecificElement<std::string> : public Element{
public:
    std::string &data;
    SpecificElement(string _key,std::string &_data):data(_data){
        key=_key;
        type = "string";
    }
    virtual void print(){
        cout<<"<"<<key<<"> ="<<data<<endl;
    }
    virtual void setValue(std::string val)
    {
        data =val;
    }
    virtual size_t getSize()
    {
        return data.length();
    }
    virtual rclcpp::parameter::ParameterVariant getParameterVariant()
    {
        return rclcpp::parameter::ParameterVariant(key, data);
    }
    virtual rclcpp::parameter::ParameterVariant getParameterVariant(std::string prefix)
    {
        return rclcpp::parameter::ParameterVariant(prefix+"."+key, data);
    }
    virtual const void * getVoidPtr(uint8_t & length, std::string & _key,std::string &_type)
    {
        _key = key;
        _type = type;

        length = data.length();
        return data.c_str();

    }
    virtual std::vector<uint8_t> getBytes(std::string & _key, std::string &_type)
    {
        _key = key;
        _type = type;
        std::vector<uint8_t> bytes;
        bytes.reserve(data.length());
        bytes.resize(data.length());
        std::memcpy(bytes.data(),data.c_str(),data.length());
        return bytes;

    }
};

template <>
class SpecificElement<std::vector<uint8_t>> : public Element{
public:
    std::vector<uint8_t> &data;
    SpecificElement(string _key,std::vector<uint8_t> &_data):data(_data){
        key=_key;
        type = "std::vector<uint8_t>";
    }
    virtual void print(){
        cout << "vector<uint8_t> Size: " << data.size() << std::endl;
    }
    virtual void setValue(std::vector<uint8_t> val)
    {
        data =val;
    }
    virtual size_t getSize()
    {
        return data.size();
    }

    virtual rclcpp::parameter::ParameterVariant getParameterVariant()
    {
        return rclcpp::parameter::ParameterVariant(key, data);
    }
    virtual rclcpp::parameter::ParameterVariant getParameterVariant(std::string prefix)
    {
        return rclcpp::parameter::ParameterVariant(prefix+"."+key, data);
    }
    virtual const void * getVoidPtr(uint8_t & length, std::string & _key,std::string &_type)
    {
        _key = key;
        _type = type;
        length = data.size();
        return data.data();
    }
    virtual std::vector<uint8_t> getBytes(std::string & _key, std::string &_type)
    {
        _key = key;
        _type = type;
        return data;
    }
};

