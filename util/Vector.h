#ifndef VECTOR_H
#define VECTOR_H
#include <QDataStream>
#include <QDebug>

// The Vector class wraps a std::vector and provides for it a nicer, Qt compatible interface.
// std::vector has two benefits over the Qt containers:
// 1. The Vector is memory persistant. When you clear() it, it does not free its memory, i.e.
// the capacity() remains the same. When the container is filled again, it reuses already
// allocated memory. This improves the runtimes of time-critical programs quite a bit.
// As a difference to std::vector, Vector does not destroy the contained objects when cleared.
// 2. The default assignment performs a deep copy of the data. Qt's implicit sharing is nice,
// but it results in unexpected runtime peaks when you don't code carefully enough. A deep copy
// on assignment is most explicit.

template <typename T>
class Vector
{
    std::vector<T> d;
    int tailIdx = -1; // -1 indicates that the list is empty. Because of this, the size is an int.

public:

    Vector() {}
    Vector(int k) {resize(k);}

    int size() const {return tailIdx+1;}
    int length() const {return size();}
    void resize(int k) {d.resize(k);tailIdx=k-1;}
    void reserve(int k) {d.reserve(k);}
    void ensureSize(int k) {if (size() < k) resize(k);}
    int capacity() const {return d.capacity();}
    bool isEmpty() const {return (tailIdx == -1);}
    bool empty() const {return isEmpty();}
    void clear() {tailIdx=-1;}

    void fill(const T& v)
    {
        for (int i = 0; i < size(); i++)
            d[i] = v;
    }

    void push_front(T const& e)
    {
        d.insert(d.begin(), e);
        tailIdx++;
    }
    void push_back(T const& e)
    {
        tailIdx++;
        if (tailIdx < d.size())
        {
            // Reuse already allocated memory (fast).
            d[tailIdx] = e;
        }
        else
        {
            // Allocate new memory (slow).
            d.push_back(e);
        }
    }
    void append(T const& e) {push_back(e);}
    void prepend(T const& e) {push_front(e);}
    Vector<T>& operator<<(T const& e) {push_back(e); return *this;}
    void insert(int i, T const& e) {d.insert(i, e);tailIdx++;}

    T& first() {return d[0];}
    T& last() {return d[size()-1];}
    T& random() {return d[(rand()/RAND_MAX)*size()];}
    const T& first() const {return d[0];}
    const T& last() const {return d[size()-1];}

    T pop_front() {T e = d[0]; d.erase(d.begin()); return e;}
    T pop_back() {T e = d[size()-1]; d.pop_back(); return e;}
    T takeFirst() {return pop_front();}
    T takeLast() {return pop_back();}
    T removeLast() {return pop_back();}

    const T* data() const {return d.data();}

    void swap(uint i, uint j) {T e = d[i]; d[i]=d[j]; d[j]=e;}
    void removeAt(uint i) {remove(i);}
    void remove(uint i) {d.erase(d.begin()+i);tailIdx--;}
    void removeAll(const T& t)
    {
        for (int i = size()-1; i >= 0; i--)
            if (d[i] == t)
                remove(i);
    }
    void removeOne(const T& t)
    {
        for (int i = 0; i < size(); i++)
        {
            if (d[i] == t)
            {
                remove(i);
                return;
            }
        }
    }

    T& operator[](uint i) {return d[i];}
    const T& operator[](uint i) const {return d[i];}
    const T& at(uint i) const {return d[i];}
    bool contains(const T& t) const
    {
        for (int i = 0; i < size(); i++)
            if (d[i] == t)
                return true;
        return false;
    }

    void sort(int direction=1)
    {
        if (direction < 0)
            std::sort(d.begin(), d.end(), std::greater<T>());
        else
            std::sort(d.begin(), d.end());
    }

    void streamOut(QDataStream& out) const
    {
        out << size();
        for (int i = 0; i < size(); i++)
            out << d[i];
    }

    void streamIn(QDataStream &in)
    {
        uint k;
        in >> k;
        resize(k);
        for (int i=0; i < k; i++)
            in >> d[i];
    }

    void operator<<(const Vector<T> &o)
    {
        for (int i=0; i < o.size(); i++)
            push_back(o[i]);
    }

    void operator<<(const std::vector<T> &o)
    {
        for (int i=0; i < o.size(); i++)
            push_back(o[i]);
    }
};

// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, const Vector<T> &o)
{
    dbg << "[";
    if (o.size() > 0)
    {
        dbg << o[0];
        for (int i = 1; i < o.size(); i++)
            dbg << "," << o[i];
    }
    dbg << "]";

    return dbg;
}

template <typename T>
QDataStream& operator<<(QDataStream& out, const Vector<T> &o)
{
    o.streamOut(out);
    return out;
}

template <typename T>
QDataStream& operator>>(QDataStream& in, Vector<T> &o)
{
    o.streamIn(in);
    return in;
}

#endif
