#include <string>
#include <stdio.h>
#include <complex>
#include <vector>
#include <iostream>
#include <sstream>

using std::vector;
using namespace std;

#define uint8 unsigned char
#define uint32 unsigned int

struct segmentcfg
{
    uint32 segid_s;
    uint32 segmentnum;
    vector<uint32> presegid;
    vector<uint32> nextsegid;
    vector<uint32> mark;
};
struct pointcfg
{
    uint32 pointid;
    vector<uint32> seg_conn;
};
const string NumTostring(int num)
{  
    ostringstream oss;  
    oss << num;
    return oss.str();      
}
int main(int argc, char* argv[])
{
    //strat segment number
    int start_segment_num = 122;

    FILE* fp = fopen("./cfg/input.txt","r");
	if(NULL == fp)
	{
	    //cout<<"can not open cfg!"<<endl;
		return false;
	}

    vector<vector<uint32>> index;
    vector<segmentcfg> out;
    char ch[100];

    //get input list
    char * pch;
    uint32 tmp;
    while(!feof(fp))
    {
        fscanf(fp,"%s\n",ch);
        vector<uint32> tmp_v;
        pch = strtok (ch," ,");
        while(pch != NULL)
        {
            string str = pch;
            istringstream iss(str);
            iss >> tmp; 
            tmp_v.push_back(tmp);
            pch = strtok (NULL, " ,");
        }
        index.push_back(tmp_v);
    }
    fclose(fp);

    //analysis input , generate the segmentid , segmentnum,presegid,nextsegid,placemark
    int i = 0;
    int size = start_segment_num - 1;
    while(i < index.size())
    {
        segmentcfg cfg_tmp;
        cfg_tmp.segid_s = size + 1;
        cfg_tmp.segmentnum = index[i].size()-1;
        cfg_tmp.mark = index[i];

        size = size + index[i].size()-1;

        out.push_back(cfg_tmp);
        i++;
    }

    //pre , next
    i=0;
    while(i < index.size())
    {
        int prepoint = out[i].mark.front();//pre
        int nextpoint = out[i].mark.back();//next
        //travels all segment
        int j = 0;
        while(j < index.size())
        {
            //if((prepoint == out[j].mark.back())||(prepoint == out[j].mark.front()))//s-s , s-e
            if(prepoint == out[j].mark.back())//s-e
            {
                if(out[i].segid_s != out[j].segid_s)
                {
                    out[i].presegid.push_back(out[j].segid_s+out[j].segmentnum-1);
                }
            }
            else if(prepoint == out[j].mark.front())//s-s
            {
                if(out[i].segid_s != out[j].segid_s)
                {
                    out[i].presegid.push_back(0);
                }
            }
            if(nextpoint == out[j].mark.front()) //e-s 
            {
                if(out[i].segid_s != out[j].segid_s)
                {
                    out[i].nextsegid.push_back(out[j].segid_s);
                }
            }
            else if(nextpoint == out[j].mark.back())//e-e
            {
                if(out[i].segid_s != out[j].segid_s)
                {
                    out[i].nextsegid.push_back(0);
                }           
            }
            j++;
        }
        i++;
    }

    //generate output cfg && FileIndex
    i = 0;
    string s = "./cfg/cfg_segment.txt";
    string s_tmp;
    string num;
    FILE* fp2 = fopen("./cfg/FileIndex.txt","w+");
    if(NULL == fp2)
    {
        cout<<"error!"<<endl;
        return 0;
    }

    while(i < index.size())
    {
        //change input name
        s_tmp = s;
        num = NumTostring(i);
        s_tmp.insert(17,num);

        //input FileIndex
        fprintf(fp2,"%s\n",s_tmp.c_str());

        FILE* fp1 = fopen(s_tmp.c_str(),"w+");
        if(NULL == fp1)
        {
            fclose(fp2);
            cout<<"error!"<<endl;
            return 0;
        }

        fprintf(fp1,"StartSegmentID:%d\n",out[i].segid_s);//start segid 
        fprintf(fp1,"segmentnum:%d\n",out[i].segmentnum);//seg number
        //preid
        fprintf(fp1,"PreSegmentID:");
        if(!out[i].presegid.empty())
        {
            for(int j=0;j<out[i].presegid.size()-1;j++)
            {fprintf(fp1,"%d,",out[i].presegid[j]);}
            fprintf(fp1,"%d\n",out[i].presegid.back());
        }
        else
        {
            fprintf(fp1,"0\n");
        }
        //nextid
        fprintf(fp1,"NextSegmentID:");
        if(!out[i].nextsegid.empty())
        {
            for(int j=0;j<out[i].nextsegid.size()-1;j++)
            {fprintf(fp1,"%d,",out[i].nextsegid[j]);}
            fprintf(fp1,"%d\n",out[i].nextsegid.back());
        }
        else
        {
            fprintf(fp1,"0\n");
        }
        //placemark
        fprintf(fp1,"PlaceMark:");
        for(int j=0;j<out[i].mark.size()-1;j++)
        {fprintf(fp1,"%d,",out[i].mark[j]);}
        fprintf(fp1,"%d\n",out[i].mark.back());
        //lanenum:
        fprintf(fp1,"lane_num:\n");
        //linenum
        fprintf(fp1,"line_num:\n");
        //LineTypeIndex
        fprintf(fp1,"LineTypeIndex:\n");
        //LaneDrectionIndex
        fprintf(fp1,"LaneDrectionIndex:\n");
        //LaneAssign
        fprintf(fp1,"LaneAssign:\n");
        //LineConn
        for(int j=0;j<out[i].nextsegid.size();j++)
        {fprintf(fp1,"LineConn:\n");}

        fclose(fp1);
        i++; 
    }

    fclose(fp2);

    return 0;
}