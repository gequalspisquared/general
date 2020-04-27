// har har ternary search for kOS
#include <stdio.h>
#include <iostream>

int ternarySearch(int initial, int len, int key, int array[]) {
    if(len >= 1) {
        int mid1 = initial + (len - 1) / 3;
        int mid2 = len - (len - 1) / 3;

        if(array[mid1] == key) {
            return mid1;
        }
        if(array[mid2] == key) {
            return mid2;
        }

        if(key < array[mid1]) {
            return ternarySearch(initial, mid1 - 1, key, array);
        } 
        else if (key > array[mid2]) {
            return ternarySearch(mid2 + 1, len, key, array);
        } 
        else {
            return ternarySearch(mid1 + 1, mid2 - 1, key, array);
        }
    }

    return -1; // key not found
}

int main() {
    int array[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    int len = (sizeof(array) / sizeof(int)) - 1;
    int start = 0; // starting index
    int key = 5;

    int index = ternarySearch(start, len, key, array);
    printf("Index of %d is %d.\n", key, index);

    printf("this shit still works right?\n");
    std::cin.get();
    return 0;
}