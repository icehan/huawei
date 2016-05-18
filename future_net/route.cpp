#include "route.h"
#include "lib_record.h"
#include <stdio.h>
#include "CommonStruct.h"
#include "ListBfsDijstra.h"
#include "StupidFind.h"
#include <iostream>
using namespace std;
//你要完成的功能总入口
void search_route(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num)
{
    StupidFind sf(topo, edge_num, demand, demand_num);
    st_sf_path m, b;
    sf.run(m, b);
    
    for (auto e: m.sides){
        record_result(WORK_PATH, e);
    }
    for (auto e: b.sides){
        record_result(BACK_PATH, e);
    }
}
