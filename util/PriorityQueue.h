#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H
#include <vector>
#include <type_traits>
#include <functional>
#include <QDebug>

// This is a priority queue implementation with a bit of extra functionality.
// Unlike the std::priority_queue, this implementation supports the "decrease key"
// function. It happens automatically when you push() an element and the element
// is already in the queue. This priority queue also allows to set a limit on the
// size of the queue with the setLimit() function. There is no limit set by default.
// If you do set a limit, pushing an item that exceeds the limit results in the
// item being sorted into the priority queue and then the last item in the queue
// being discarded. The PriorityQueue is a template class so that you can use it
// basically with any object, but the queue will work only on pointers! Here is
// a usage example:
//
// Node* node1;
// Node* node2;
// PriorityQueue<Node*> q;
// q.push(node1);
// q.push(node2);
// Node* min = q.top();
// q.pop();
//
// Furthermore, the objects must implement a cmp(T*), a setIdx(uint) and the
// uint getIdx() functions in order to be able to work with the queue.



template <typename T>
class PriorityQueue
{
    int limit;
    std::vector<T> d;

public:
    PriorityQueue();
    void setLimit(uint l);
    void push(T);  // push x
    const T top() const; // return top element
    void pop(); // pop element
    uint size() const {return d.size()-1;}
    bool empty() const {return d.size() <= 1;}
    bool isEmpty() const {return empty();}
    void clear();
    const T operator[](int i) const {return d[i+1];}

private:
    void swap(uint i, uint j);
};

template <typename T>
PriorityQueue<T>::PriorityQueue()
{
    limit = 0;
    d.push_back(T());
}

// Imposes a limit on the size of the queue and discards excess items from the back
// of the queue. This is a slow operation because it may allocate new memory if
// the new limit is higher than the old one. It's best when you set the limit before
// you start sorting items into the pq.
template <typename T>
void PriorityQueue<T>::setLimit(uint l)
{
    if (l == limit)
        return;
    limit = l;
    while (limit > 0 && d.size()+1 > limit)
        d.pop_back();
    d.reserve(limit);
}

// Removes all items from the queue.
template <typename T>
void PriorityQueue<T>::clear()
{
    d.clear();
    d.push_back(T());
}

// Sorts a new item into the queue in O(log N) time.
// If a size limit is set and the queue is larger than the limit,
// it will truncate the queue by throwing away the last element.
template <typename T>
void PriorityQueue<T>::push(T t)
{
    //qDebug() << "      " << "Pushing" << t->f << "idx:" << t->getIdx();

    uint i;
    if (t->getIdx() > 0)
    {
        i = t->getIdx();
    }
    else
    {
        d.push_back(t);
        i = d.size()-1;
    }

    // Bubble up.
    while (i > 1 && t->cmp(d[i/2]))
    {
        d[i] = d[i/2];
        d[i]->setIdx(i);
        i = i/2;
    }

    d[i] = t;
    d[i]->setIdx(i);

    // Truncate to limit.
    if (limit > 2 && d.size()-1 > limit)
        d.pop_back(); // Does not always delete the largest element, but it's simple and fast.
}

// Pops the first item from the queue. No item is returned!
template <typename T>
void PriorityQueue<T>::pop()
{
    uint s = d.size()-1;
    uint l,r,smallest;
    uint i = 1;

    d[1]->setIdx(0);

    if (s == 1)
    {
        d.pop_back();
        return;
    }

    // Bubble down.
    while (true)
    {
        l = 2*i;
        r = l+1;
        smallest = s;

        if (l < s && d[l]->cmp(d[smallest]))
            smallest = l;
        if (r < s && d[r]->cmp(d[smallest]))
            smallest = r;

        d[i] = d[smallest];
        d[i]->setIdx(i);

        if (smallest != s)
        {
            i = smallest;
        }
        else
        {
            d.pop_back();
            return;
        }
    }
}

// Returns a const reference to the first item in the queue.
// This will segfault if you call it on an empty queue.
template <typename T>
const T PriorityQueue<T>::top() const
{
    return d[1]; // Hmpf. What if there is none?
}

// Swaps element j with element i.
template <typename T>
void PriorityQueue<T>::swap(uint i,  uint j)
{
    T tmp = d[i];
    d[i] = d[j];
    d[j] = tmp;
    d[i]->setIdx(i);
    d[j]->setIdx(j);
}

// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, PriorityQueue<T> &o)
{
    bool sp = dbg.autoInsertSpaces();
    dbg.nospace();
    dbg << "[";
    if (o.size() > 0)
    {
        dbg << o[0]->id << ":" << o[0]->f << ":" << o[0]->getIdx();
        for (int i = 1; i < o.size(); i++)
            dbg << ", " << o[i]->id << ":" << o[i]->f << ":" << o[i]->getIdx();
    }
    dbg << "] ";
    dbg.setAutoInsertSpaces(sp);
    return dbg;
}

#endif
