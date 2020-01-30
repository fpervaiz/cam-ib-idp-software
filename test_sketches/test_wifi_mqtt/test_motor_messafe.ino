#include <SPI.h>
#include <PubSubClient.h>
#include <WiFiNINA.h>

// New preamble
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>

void useWifiMessage(char payload){
    
    const char* token = ",";
    std::vector<char*> v;
    
    char* chars_array = strtok(&payload, token);
    
    while(chars_array){
        v.push_back(chars_array);
        chars_array = strtok(NULL, ",");
    }

    for (size_t n = 0; n < v.size(); ++n){
        char* subchar_array = strtok(v[n], ":");
        while(subchar_array){
            std::cout << subchar_array<< "\n";
        }
    }
}

int main(){
    char pl[] = "hello,goodbye";
    useWifiMessage((char)pl);
}