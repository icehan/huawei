#ifndef __ICEMINPQ__H_
#define __ICEMINPQ__H_

#include<vector>
#include<algorithm>

template<typename Key, typename Val>
class IceMinPQ
{
public:
    bool empty()
    {
        return data.empty();
    }
    Key  top()	
    {
        return data[0].K;
    }
    void pop()
    {
        std::pop_heap(data.begin(), data.end()); 
        data.pop_back();
    }
    void insert(Key k, Val v)
    {
        data.push_back(pair(k,v)); 
        std::push_heap(data.begin(), data.end());
    }
    bool contains(Key k)
    {
        for (size_t i = 0; i < data.size(); i++) 
            if (data[i].K == k) 
                return true; 
        return false;
    }
    void change(Key k, Val v)
    {
        for (size_t i = 0; i < data.size(); i++) 
        if (data[i].K == k) {
            data[i].V = v; 
            break;
        }
    }
private:
	struct pair{
	    Key K; Val V;
	    pair(Key k, Val v):K(k),V(v){}
	    bool operator<(const pair &p){return V > p.V;}
    };
    std::vector<pair> data;
};

#endif
