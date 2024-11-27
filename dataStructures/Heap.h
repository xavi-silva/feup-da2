#ifndef UNTITLED_HEAP_H
#define UNTITLED_HEAP_H

#include <vector>

class Heap {
public:
    Heap();
    Heap(std::vector<int> v);
    void insert(int x);
    int extractMin();
    bool empty();
private:
    std::vector<int> elems;
    void heapifyUp(unsigned int i);
    void heapifyDown(unsigned int i);
};
#endif //UNTITLED_HEAP_H
