#ifndef LINKEDLIST_H_
#define LINKEDLIST_H_
#include <QDataStream>
#include <QDebug>

// This is a memory preserving LinkedList implementation designed for robot control
// applications where typically the list is often cleared and refilled with new data.
// Since the allocation of memory is a performance bottleneck, memory allocation is
// avoided in a way that heap memory is allocated only for new elements that exceed
// the capacity of the list, but the memory is never released. When an item is deleted
// from the list, or the list is cleared, only its size is decremented or reset to zero,
// but no memory is released. It is reused for faster insertion of future elements
// instead. This way the capacity of the list only ever grows and never shrinks.
// The reserve() method can be used to allocate memory for a known number of items.
// This way, appending items up to the reserved size will be fast.

// The LinkedList offers a std library-compatible interface to access, add, and remove
// items at the begining and the end of the list. To navigate the list, use the iterator
// interface begin() and end() to obtain a ListIterator that you can then use to step
// through the list in forward (next()) and backward (prev()) directions. The ListIterator
// is also able to wrap around if you need to loop over all items multiple times.
// Random access is not supported.

/* copy paste snippet
LinkedList<Node> nodes;
nodes << Node();
ListIterator<Node> it = nodes.begin();
while (it.hasNext())
{
    Node& node = it.next();
}

*/

// One item of the linked list with pointers to the next and the previous element in the list.
template <typename T>
struct ListItem
{
    T d;
    ListItem<T>* next=0;
    ListItem<T>* prev=0;
};

// This iterator allows you to iterate through the list from head to tail and even to cycle
// through the list over and over again. hasNext() will be false when the last item in the
// list is reached, but next() can still be called and it will reset to the head.
template <typename T>
struct ListIterator
{
    ListItem<T>* const* head=0;
    ListItem<T>* const* tail=0;
    ListItem<T>* cur_=0;
    bool flipped=false;

public:

    // Tells you if the iterator has one more element to move forward to.
    bool hasNext() const
    {
        return !flipped;
    }

    // Tells you if the iterator has one more element to move backward to.
    bool hasPrev() const
    {
        return !flipped;
    }

    // Returns true if the iterator is pointing at the tail.
    bool atEnd() const
    {
        return (cur_ == *tail);
    }

    // Returns true if the iterator is pointing at the head.
    bool atStart() const
    {
        return (cur_ == *head);
    }

    // Returns the current element by reference and moves forward to the next one.
    // If the current one is the last element in the list, the iterator is automatically
    // reset to the head.
    T& next()
    {
        if (cur_ == *tail)
        {
            cur_ = *head;
            flipped = true;
            return (*tail)->d;
        }
        else
        {
            cur_ = cur_->next;
            flipped = false;
            return cur_->prev->d;
        }
    }

    // Returns the current element and moves backward to the previous one.
    // If the current one is the first element in the list, the iterator
    // is reset to the tail.
    T& prev()
    {
        if (cur_ == *head)
        {
            cur_ = *tail;
            flipped = true;
            return (*head)->d;
        }
        else
        {
            cur_ = cur_->prev;
            flipped = false;
            return cur_->next->d;
        }
    }

    // Returns the current element without modifying the iterator.
    T& peekCur() const
    {
        return cur_->d;
    }

    // Returns the previous element without modifying the iterator.
    T& peekPrev() const
    {
        if (cur_ == *head)
            return (*tail)->d;
        return cur_->prev->d;
    }

    // Returns the next element without modifying the iterator.
    T& peekNext() const
    {
        if (cur_ == *tail)
            return (*head)->d;
        return cur_->next->d;
    }
};

template <typename T>
class LinkedList
{
    ListItem<T>* head; // Pointer to the first element in the list.
    ListItem<T>* tail; // Pointer to the last element in the list.
    int size_; // How many elements are actually in the list.
    int capacity_; // How many memory slots have been allocated.
    ListIterator<T> it; // An internal list iterator to support easier element access.

public:

    LinkedList()
    {
        head = new ListItem<T>();
        tail = head;
        size_ = 0;
        capacity_ = 1;
        it = begin();
    }

    ~LinkedList()
    {
        ListItem<T>* cur = head;
        while (cur != tail)
        {
            cur = cur->next;
            delete cur->prev;
        }
        delete tail;
    }

    // Copy constructor.
    LinkedList(const LinkedList &o)
    {
        head = new ListItem<T>();
        tail = head;
        size_ = 0;
        capacity_ = 1;
        *this = o;
    }

    // Assignment operator.
    // Assignment of one linked list to another creates a deep copy.
    // If the list to be copied is smaller than this one, already allocated memory is preserved.
    // Otherwise this list grows in size to accomodate the other list.
    LinkedList<T>& operator=(const LinkedList<T> &o)
    {
        if (this == &o)
            return *this;

        clear();

        ListIterator<T> it = o.begin();
        while (it.hasNext())
        {
            T& n = it.next();
            push_back(n);
        }

        return *this;
    }

    // Clears the list, but does not release already allocated memory.
    void clear()
    {
        size_ = 0;
        tail = head;
        it = begin();
    }

    // Reserves space in memory for k items. The memory is reserved at the back of the list.
    void reserve(int k)
    {
        if (k <= capacity_)
            return;

        ListItem<T>* cur = tail;
        for (int i = capacity_; i < k; i++)
        {
            cur->next = new ListItem<T>();
            cur->next->prev = cur;
            cur = cur->next;
        }

        capacity_ = k;
    }

    // Returns the capacity of the list, i.e. for how many items memory
    // has been reserved. This is not necessarily equal the size.
    int capacity() const
    {
        return capacity_;
    }

    // Returns the number of items in the list.
    int size() const
    {
        return size_;
    }

    // Tells you if the list is empty or not.
    bool isEmpty() const
    {
        return (size_ == 0);
    }

    // Tells you if the list is empty or not.
    bool empty() const
    {
        return isEmpty();
    }

    // Prepends a new item to the front of the list.
    // Allocated slots in memory at the front of the list are reused.
    void push_front(const T& e)
    {
        // List is empty case.
        if (size_ == 0)
        {
            head->d = e;
            size_++;
        }

        // Reusing memory case.
        else if (head->prev != 0)
        {
            head = head->prev;
            head->d = e;
            size_++;
        }

        // Allocating new memory case.
        else
        {
            head->prev = new ListItem<T>();
            head->prev->next = head;
            head = head->prev;
            capacity_++;
            head->d = e;
            size_++;
        }
    }

    // Appends a new item to the back of the list.
    // This is a very fast operation when the capacity exceeds the size.
    void push_back(const T& e)
    {
        // List is empty case.
        if (size_ == 0)
        {
            head->d = e;
            size_++;
        }

        // Reusing memory case.
        else if (tail->next != 0)
        {
            tail = tail->next;
            tail->d = e;
            size_++;
        }

        // Allocating new memory case.
        else
        {
            tail->next = new ListItem<T>();
            tail->next->prev = tail;
            tail = tail->next;
            tail->d = e;
            capacity_++;
            size_++;
        }
    }

    // Appends a new item to the back of the list.
    LinkedList<T>& operator<<(const T& e)
    {
        push_back(e);
        return *this;
    }

    // Appends a new item to the back of the list.
    void push(const T& e)
    {
        push_back(e);
    }

    // Removes the first item from the list.
    T pop_front()
    {
        // Empty list case.
        if (size_ == 0)
            return head->d;

        // One item case.
        if (head == tail)
        {
            T d = head->d;
            head->d = T();
            size_ = 0;
            return d;
        }

        head = head->next;
        size_--;
        return head->prev->d;
    }

    // Removes the last item from the List.
    T pop_back()
    {
        // Empty list case.
        if (size_ == 0)
            return head->d;

        // One item case.
        if (head == tail)
        {
            T t = head->d;
            head->d = T();
            size_ = 0;
            return t;
        }

        tail = tail->prev;
        size_--;
        return tail->next->d;
    }

    // Removes the last item from the List.
    T pop()
    {
        return pop_back();
    }

    // Removes all elements from the linked list that evaluate the == operator
    // to true with the given element.
    void remove(const T& d)
    {
        // The current implementation is not memory preserving.

        // Empty list case.
        if (size() == 0)
            return;

        if (head->d == d)
        {
            pop_front();
            return;
        }
        if (tail->d == d)
        {
            pop_back();
            return;
        }

        if (head != tail)
        {
            ListItem<T>* cur = head->next;

            while (cur != tail)
            {
                if (cur->d == d)
                {
                    cur->prev->next = cur->next;
                    cur->next->prev = cur->prev;
                    ListItem<T>* del = cur;
                    cur = cur->next;
                    delete del;
                    size_--;
                }
                else
                {
                    cur = cur->next;
                }
            }
        }
    }

    // Returns true if the list contains at least one element that evaluates
    // the == operator to true with the given element.
    bool contains(const T& d)
    {
        // Empty list case.
        if (size() == 0)
            return false;

        ListItem<T>* cur = head;

        // One element case.
        if (head == tail)
        {
            if (head->d == d)
                return true;
        }
        else
        {
            while (cur != tail)
            {
                if (cur->d == d)
                    return true;
                cur = cur->next;
            }
        }

        return false;
    }

    // Returns the first element in the list (the head) without modifying the list.
    T& first() const
    {
        return head->d;
    }

    // Returns the last elemet in the list (the tail) without modifying the list.
    T& last() const
    {
        return tail->d;
    }

    // The same as last().
    T& top() const
    {
        return last();
    }

    // Returns an iterator pointing at the head of the list.
    ListIterator<T> begin() const
    {
        ListIterator<T> it;
        it.head = &(head); // Taking a pointer to the head pointer here makes the iterator more robust to changes of the list while iterating over it.
        it.tail = &(tail);
        it.cur_ = head;
        it.flipped = (size_ == 0);
        return it;
    }

    // Returns an iterator pointing at the end of the list.
    ListIterator<T> end() const
    {
        ListIterator<T> it;
        it.head = &(head);
        it.tail = &(tail);
        it.cur_ = tail;
        it.flipped = (size_ == 0);
        return it;
    }

    // Sorts the list in ascending order using the < operator.
    // If you pass a -1 as the argument, it will sort in reverse order.
    void sort(int direction=0)
    {
        // One item case.
        if (head == tail)
            return;

        ListItem<T> dummy;
        dummy.next = head;
        dummy.prev = head->prev;
        tail = mergesort(&dummy, size_, direction);
        head = dummy.next;
        head->prev = dummy.prev;

        // Fix the prev pointers.
        ListItem<T> *cur = head;
        while (cur != tail)
        {
            cur->next->prev = cur;
            cur = cur->next;
        }
    }

    // Reverses the order of the items in the list. The tail becomes the head and the head becomes the tail.
    void reverse()
    {
        ListItem<T> *cur = head;
        ListItem<T> *tmp;
        while (cur != tail)
        {
            tmp = cur->next;
            cur->next = cur->prev;
            cur->prev = tmp;
            cur = cur->prev;
        }

        tmp = tail->next;
        tail->next = tail->prev;
        tail->prev = tmp;

        tmp = head;
        head = tail;
        tail = tmp;
    }


    // Tells you if the iterator has one more element to move forward to.
    bool hasNext() const
    {
        return it.hasNext();
    }

    // Tells you if the iterator has one more element to move backward to.
    bool hasPrev() const
    {
        return it.hasPrev();
    }

    // Returns true if the iterator is pointing at the tail.
    bool atEnd() const
    {
        return it.atEnd();
    }

    // Returns true if the iterator is pointing at the head.
    bool atStart() const
    {
        return it.atStart();
    }

    // Returns the current element by reference and moves forward to the next one.
    // If the current one is the last element in the list, the iterator is automatically
    // reset to the head.
    T& next()
    {
        return it.next();
    }

    // Returns the current element and moves backward to the previous one.
    // If the current one is the first element in the list, the iterator
    // is reset to the tail.
    T& prev()
    {
        return it.prev();
    }

    // Returns the current element without modifying the iterator.
    T& peekCur() const
    {
        return it.peekCur();
    }

    // Returns the previous element without modifying the iterator.
    T& peekPrev() const
    {
        return it.peekPrev();
    }

    // Returns the next element without modifying the iterator.
    T& peekNext() const
    {
        return it.peekNext();
    }

    // Resets the internal iterator to the head of the list.
    // This will not clear the linked list. Use clear() for that.
    void reset()
    {
        it = begin();
    }

private:
    ListItem<T>* mergesort(ListItem<T> *start, long lengtho, int direction)
    {
        long count1=(lengtho/2), count2=(lengtho-count1);
        ListItem<T> *next1,*next2,*tail1,*tail2,*tail;
        if (lengtho<=1) return start->next;  /* Trivial case. */
        tail1 = mergesort(start, count1, direction);
        tail2 = mergesort(tail1, count2, direction);
        tail = start;
        next1 = start->next;
        next2 = tail1->next;
        tail1->next = tail2->next; /* in case this ends up as the tail */
        while (1)
        {
            if(direction+(next1->d < next2->d)) // Use of < operator.
            {
                tail->next=next1; tail=next1;
                if(--count1==0) { tail->next=next2; return tail2; }
                next1=next1->next;
            }
            else
            {
                tail->next=next2; tail=next2;
                if(--count2==0) { tail->next=next1; return tail1; }
                next2=next2->next;
            }
        }
    }

public:
    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);
};

template <typename T>
void LinkedList<T>::streamOut(QDataStream& out) const
{
    out << size_;
    if (size_ == 0)
        return;
    if (head == tail)
    {
        out << head->d;
        return;
    }
    ListItem<T>* cur = head;
    while (cur != tail)
    {
        out << cur->d;
        cur = cur->next;
    }
    out << tail->d;
}

template <typename T>
void LinkedList<T>::streamIn(QDataStream &in)
{
    clear();
    int howMany;
    in >> howMany;
    for (int i = 0; i < howMany; i++)
    {
        T e;
        in >> e;
        push_back(e);
    }
}

template <typename T>
QDataStream& operator<<(QDataStream& out, const LinkedList<T> &o)
{
    o.streamOut(out);
    return out;
}

template <typename T>
QDataStream& operator>>(QDataStream& in, LinkedList<T> &o)
{
    o.streamIn(in);
    return in;
}

// QDebug output.
template <typename T>
QDebug operator<<(QDebug dbg, const LinkedList<T> &o)
{
    bool sp = dbg.autoInsertSpaces();
    dbg.nospace();
    dbg << "[";
    if (o.size() > 0)
    {
        ListIterator<T> it = o.begin();
        dbg << it.next();
        while (it.hasNext())
            dbg << ", " << it.next();
    }
    dbg << "] ";
    dbg.setAutoInsertSpaces(sp);
    return dbg;
}

#endif
