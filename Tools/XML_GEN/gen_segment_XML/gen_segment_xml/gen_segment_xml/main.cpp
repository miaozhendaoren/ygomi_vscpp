#include <string>
#include <stdio.h>
#include <complex>
#include <vector>
#include <iostream>
#include <sstream>
#include "XMLParser.h"

using std::vector;
using namespace std;

#define uint8 unsigned char
#define uint32 unsigned int
#define MAX_LINE_NUM 20
#define MAX_LANE_NUM 5
#define MAX_CONN_SEGMENT_NUM 4
#define EMPTY  0
#define FULL   1
#define IDLE   -1
#define MERGE   1
#define NOTMERGE 0
#define MAX_FILENAME_LENTH 40
#define MAX_FILE_NUM 50
#define REFERENCE_LAT "48.350662000000000"
#define REFERENCE_LON "11.733637999999999"

vector<uint32> LineTypeIndex;
vector<uint32> LaneDrectionIndex;
vector<uint32> LaneAssign;
vector<uint32> NextSegmentID;
vector<uint32> PreSegmentID;
vector<uint32> PlaceMark;
struct conn_info_t
{
    uint32 connlineS; //last segment conn start line number
    uint32 connNextlineS;// last segment conn seg start line number
    uint32 connlineNum;
    uint32 connsegid;
};
vector<conn_info_t> conn_info_g;

struct filename_node
{
    char name[MAX_FILENAME_LENTH];
	filename_node *p_next;
};
struct point3D_t
{
    string lat;
    string lon;
    string alt;
};
class Segmentpoint
{
public:
    uint32 pointId;
    uint32 point_type;
    vector<uint32> conn_segId;
    point3D_t point;
};

class Segmentlane
{
public:
    uint32  laneId;		  
	uint32  derection;
	uint32  merge_left;
	uint32  merge_right;
	uint32  startpointID;
    uint32  endpointID;
	uint32  leftlineID;
	uint32  rightlineID; 
};

struct lineconn_seg_info
{
    uint32 segid;
	uint32 lineid;
};
class Segmentline
{
public:
	uint32 lineId;
	uint32 linetype;
	//vector<uint32> connLineID;
	//vector<uint32> connSegID;
	vector<lineconn_seg_info> conninfo;
};

class RoadSegment
{
public:
    uint32 segId;
    uint32 segtype;
	vector<Segmentpoint> _segpoint;
    vector<Segmentlane> _seglane;
    vector<Segmentline> _segline;
};
class All_RoadSegment
{
public:	
    uint32 segmentstartId;
    uint32  segmentnum;
    //uint32  segType;
    uint32  laneNum;
	uint32  lineNum;
	uint32  pointNum;
    /*all the segments are the same type ,lane,line..*/
    vector<RoadSegment> _roadsegment;
};

template <class Type>  
const string NumTostring(Type num)
{  
    ostringstream oss;  
    oss << num;
    return oss.str();      
} 

bool filterlineInfo(uint32 type , vector<conn_info_t> &conn_info) //type :0 pre 1:next
{
    if(type > 2)
    {
        printf("input error!\n");
		return false;
	}

	vector<uint32> tmp;
	if(0 == type)
	{
        tmp = PreSegmentID;
	}
	else
	{
        tmp = NextSegmentID;
	}
	
    uint32 i = 0;
    //check segment id
    while(i < conn_info_g.size())
    {
        for(uint32 j=0;j < tmp.size();j++)
        {
            if(tmp[j] == conn_info_g[i].connsegid)
            {
				conn_info.push_back(conn_info_g[i]);
			}
		}
        i++;
	}

	return true;
}
	
void CfgSegments(All_RoadSegment *allsegment)
{
    int cnt=0;
    for(int j=0;j<LaneAssign.size();j++)
    {
        if(1 == LaneAssign[j])
        {
            cnt++;
        }
    }
    if(cnt!=allsegment->laneNum) 
    {
        cout<<"input Error!"<<endl;
        return ;
    }

    Segmentlane seglane_tmp[MAX_LINE_NUM-1];
    int prestaus = IDLE;
    /*not merge:0 merge:1*/
    for(int i=0;i<LaneAssign.size();i++)
    {
        if(0 == i)
        {
            seglane_tmp[0].merge_left = NOTMERGE;
        }

        if(FULL == LaneAssign[i])
        {
            if(EMPTY == prestaus)
            {
                seglane_tmp[i].merge_left = NOTMERGE;
            }
            else if(FULL == prestaus)
            {
                seglane_tmp[i].merge_left = MERGE;
                seglane_tmp[i-1].merge_right = MERGE; 
            }
            prestaus = FULL;
        }
        else
        {
            if(EMPTY == prestaus)
            {}
            else if(FULL == prestaus)
            {
                seglane_tmp[i-1].merge_right = NOTMERGE;
            }
            prestaus = EMPTY;
        }

        if(i == (LaneAssign.size()-1))
        {
            seglane_tmp[i].merge_right = NOTMERGE;
        }
                
    }

    for(int idx=0;idx<allsegment->segmentnum;idx++)
    {
        RoadSegment roadsegment_push;
        roadsegment_push.segId = idx + allsegment->segmentstartId;//start ID
        roadsegment_push.segtype = 0;
        /*lane*/
        cnt=0;
        for(int j=0;j<allsegment->laneNum;j++)
        {
            Segmentlane seglane_push;
            seglane_push.laneId = j+1;
            seglane_push.startpointID=0;
            seglane_push.endpointID=0;
            seglane_push.derection = LaneDrectionIndex[j];
            //while((FULL!=LaneAssign[cnt])||(cnt == MAX_LINE_NUM-1))
            while((cnt<LaneAssign.size()) && (FULL!=LaneAssign[cnt]))
            {
                cnt++;
			}

            seglane_push.merge_left = seglane_tmp[cnt].merge_left;
            seglane_push.merge_right = seglane_tmp[cnt].merge_right;
            seglane_push.leftlineID = cnt+1;
            seglane_push.rightlineID = cnt+2;
            cnt = cnt +1;
            //push
            roadsegment_push._seglane.push_back(seglane_push);
        }
		/*point*/
	    for(int j=0;j<2;j++)
		{
			Segmentpoint segpoint_push;
			if(0 == j) //pre point
			{
				segpoint_push.point_type = 0;
				segpoint_push.pointId = 1;
				if(0 == idx) //first segment
				{
					segpoint_push.conn_segId = PreSegmentID;
				}
				else
				{
				    if(roadsegment_push.segId != 0)
				    {
					    segpoint_push.conn_segId.push_back(roadsegment_push.segId-1);
				    }
				}
			}
			else //next point
			{
				segpoint_push.point_type = 1;				
				segpoint_push.pointId = 2;
				if(idx == allsegment->segmentnum-1) //last segment
				{
				    segpoint_push.conn_segId = NextSegmentID;
                }
				else
				{
				    if(roadsegment_push.segId != 0)
					{
					    segpoint_push.conn_segId.push_back(roadsegment_push.segId+1);
				    }
				}
			}
			//push
			roadsegment_push._segpoint.push_back(segpoint_push);
		}

        /*line*/
        //check
        int cnt = 0;
        for(int j=0;j<allsegment->lineNum;j++)
        {
            Segmentline segline_push;
			uint32 lineId = j+1;
            segline_push.lineId = lineId;
            segline_push.linetype = LineTypeIndex[j];
			
			//first segment or last segment
            if((0 == idx)||(allsegment->segmentnum-1 == idx))
            {
                vector<conn_info_t> conn_info;
                if(0 == idx) //first
                {
                    filterlineInfo(0 , conn_info);
				}

				if(allsegment->segmentnum-1 == idx)//last
				{
                    filterlineInfo(1 , conn_info);
				}
				
                uint32 i = 0;
				//connection
                while(i < conn_info.size())
                {
                    //serch line id matched
                    if((lineId >= conn_info[i].connlineS)
				    && (lineId <= conn_info[i].connlineS+conn_info[i].connlineNum-1))
                    {
                        lineconn_seg_info tmp;
						tmp.segid = conn_info[i].connsegid;
						tmp.lineid = conn_info[i].connNextlineS + lineId - conn_info[i].connlineS;
						segline_push.conninfo.push_back(tmp);
					}
                    i++;
				}
			}
			
			//not last segment
			if((allsegment->segmentnum-1) != idx)
			{
                lineconn_seg_info tmp;
				tmp.segid = roadsegment_push.segId+1;
				tmp.lineid = lineId;
				segline_push.conninfo.push_back(tmp);
			}
			
            roadsegment_push._segline.push_back(segline_push);
        }
        
        allsegment->_roadsegment.push_back(roadsegment_push); //push roadsegment      
    }
    return;
}

int GetGps(All_RoadSegment *allsegment)
{
     vector<string> values;    
     XMLParser xml_parser("./cfg/MyPlaces.xml");
     int result = xml_parser.getValues("Folder/Document/Folder/Placemark/Point/coordinates",&values);
     if(0 == result)
     {
         return 0;
     }

     //check 
     if((allsegment->segmentnum<=0) || (allsegment->pointNum<=0))
     {
         return 0;
     }

     //get name
     vector<string> names;
     result = xml_parser.getValues("Folder/Document/Folder/Placemark/name",&names);
     int temp = stringToNum<int>(names[0]);
     int temp1 = stringToNum<int>(names[0]);
     //check name
     for(int i=1;i<names.size();i++)
     {
         temp = stringToNum<int>(names[i]);
         if(temp!=temp1+1)
         {
             cout<<"Name error! pos:"<<temp1<<endl;
         }
         temp1 = temp;
     }

     uint32 pos=0,pos1=0,index=0;
	 for(int i=0;i<(allsegment->segmentnum);i++)
     {
         for(int j=0;j<2;j++)
         {
             /*parse:lat,lon,alt*/
             string s = values[PlaceMark[j+index]-1];
             pos = s.find(",",0);
             pos1 = s.find(",",pos+1);
             string lon;
             lon.assign(s,0,pos);
             string lat;
             lat.assign(s,pos+1,pos1-pos-1);
			 
             allsegment->_roadsegment[i]._segpoint[j].point.lon = lon;
             allsegment->_roadsegment[i]._segpoint[j].point.lat = lat;
			 allsegment->_roadsegment[i]._segpoint[j].pointId = j+1;
         }
         index++;
     }

     return 1;
}
void XML_genarate(TiXmlElement *RootElement , All_RoadSegment *all_seg)
{ 
     int seg_index =0;
     while(seg_index < all_seg->segmentnum)
     {
         TiXmlElement* segmentElement=new TiXmlElement("segment");
         //segId
         TiXmlElement* segIDElement=new TiXmlElement("segId");
         segmentElement->LinkEndChild(segIDElement);
         segIDElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index].segId).c_str()));
         /*comments*/
         segmentElement->LinkEndChild(new TiXmlComment("Seg_type 0:nromal 1:crossing 2:T road 3:default"));
         //segtypeTiXmlText
         TiXmlElement* segtypeElement=new TiXmlElement("seg_type");
         segmentElement->LinkEndChild(segtypeElement);
         segtypeElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index].segtype).c_str()));
         //lanenum
         TiXmlElement* lanenumElement=new TiXmlElement("lane_num");
         segmentElement->LinkEndChild(lanenumElement);
         lanenumElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->laneNum).c_str()));
         //linenum
         TiXmlElement* linenumElement=new TiXmlElement("line_num");
         segmentElement->LinkEndChild(linenumElement);
         linenumElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->lineNum).c_str()));
         //pointnum
         TiXmlElement* pointnumElement=new TiXmlElement("point_num");
         segmentElement->LinkEndChild(pointnumElement);
         pointnumElement->LinkEndChild(new TiXmlText("2"));

         for(int i=0;i<all_seg->pointNum;i++)
         {
             TiXmlElement* pointElement=new TiXmlElement("point");
             segmentElement->LinkEndChild(pointElement);
             //point ID
             TiXmlElement* pointIdElement=new TiXmlElement("pointID");
             pointElement->LinkEndChild(pointIdElement);
             pointIdElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._segpoint[i].pointId).c_str()));
             /*comments*/
             pointElement->LinkEndChild(new TiXmlComment("Point_type 0:pre 1:next"));
             //point type
             TiXmlElement* pointtypeElement=new TiXmlElement("point_type");
             pointElement->LinkEndChild(pointtypeElement);
             pointtypeElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._segpoint[i].point_type).c_str()));
             //connection segment number
             TiXmlElement* consegnumdElement=new TiXmlElement("conn_segnum_p");
             pointElement->LinkEndChild(consegnumdElement);
			 uint32 conn_segId_num = all_seg->_roadsegment[seg_index]._segpoint[i].conn_segId.size();
             consegnumdElement->LinkEndChild(new TiXmlText(NumTostring(conn_segId_num).c_str()));
             //connection segment
             for(int j=0;j<conn_segId_num;j++)
             {
				 TiXmlElement* consegidElement=new TiXmlElement("conn_segID");
				 pointElement->LinkEndChild(consegidElement);
				 consegidElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._segpoint[i].conn_segId[j]).c_str()));
			 }
             //lat
             TiXmlElement* pointlatElement=new TiXmlElement("lat");
             pointElement->LinkEndChild(pointlatElement);
             pointlatElement->LinkEndChild(new TiXmlText((all_seg->_roadsegment[seg_index]._segpoint[i].point.lat).c_str()));
             //lon
             TiXmlElement* pointlonElement=new TiXmlElement("lon");
             pointElement->LinkEndChild(pointlonElement);
             pointlonElement->LinkEndChild(new TiXmlText((all_seg->_roadsegment[seg_index]._segpoint[i].point.lon).c_str()));
         }
		 
         for(int i=0;i<all_seg->laneNum;i++)
         {
             TiXmlElement* laneElement=new TiXmlElement("lane");
             segmentElement->LinkEndChild(laneElement);
             //lane id
             TiXmlElement* laneIdElement=new TiXmlElement("laneID");
             laneElement->LinkEndChild(laneIdElement);
             laneIdElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._seglane[i].laneId).c_str()));
             //start point ID
             TiXmlElement* startlatElement=new TiXmlElement("start_pointID");
             laneElement->LinkEndChild(startlatElement);
             startlatElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._seglane[i].startpointID).c_str()));
             //end point ID
             TiXmlElement* endlatElement=new TiXmlElement("end_pointID");
             laneElement->LinkEndChild(endlatElement);
             endlatElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._seglane[i].endpointID).c_str()));
             /*comments*/
             laneElement->LinkEndChild(new TiXmlComment("Direction 0:start to end 1:end to start"));
             //dir
             TiXmlElement* lanedirElement=new TiXmlElement("direction");
             laneElement->LinkEndChild(lanedirElement);
             lanedirElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._seglane[i].derection).c_str()));
             /*comments*/
             laneElement->LinkEndChild(new TiXmlComment("Merge 0:not merge 1:merge"));
             //merge l
             TiXmlElement* lanemlElement=new TiXmlElement("merge_left");
             laneElement->LinkEndChild(lanemlElement);
             lanemlElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._seglane[i].merge_left).c_str()));
             //merge r
             TiXmlElement* lanemrElement=new TiXmlElement("merge_right");
             laneElement->LinkEndChild(lanemrElement);
             lanemrElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._seglane[i].merge_right).c_str()));
             //left line
             TiXmlElement* leftlineidElement=new TiXmlElement("leftlineID");
             laneElement->LinkEndChild(leftlineidElement);
             leftlineidElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._seglane[i].leftlineID).c_str()));
             //right line
             TiXmlElement* rightlineidElement=new TiXmlElement("rightlineID");
             laneElement->LinkEndChild(rightlineidElement);
             rightlineidElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._seglane[i].rightlineID).c_str()));
         }
		 
         for(int i=0;i<all_seg->lineNum;i++)
         {
             TiXmlElement* lineElement=new TiXmlElement("line");
             segmentElement->LinkEndChild(lineElement);
             //lineID
             TiXmlElement* lineidElement=new TiXmlElement("lineID");
             lineElement->LinkEndChild(lineidElement);
             lineidElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._segline[i].lineId).c_str()));
             /*comments*/
             lineElement->LinkEndChild(new TiXmlComment("Line_type 0:dotted line 1:solid line "));
             //linetype
             TiXmlElement* linetypeElement=new TiXmlElement("line_type");
             lineElement->LinkEndChild(linetypeElement);
             linetypeElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._segline[i].linetype).c_str()));
             //line connection segment number
             TiXmlElement* consegnum_l_Element=new TiXmlElement("conn_segnum_l");
             lineElement->LinkEndChild(consegnum_l_Element);
             uint32 connline_seg_num = all_seg->_roadsegment[seg_index]._segline[i].conninfo.size();
             consegnum_l_Element->LinkEndChild(new TiXmlText(NumTostring(connline_seg_num).c_str()));
             for(int j = 0;j < connline_seg_num;j++)
             {
				 //connection segmentID
				 TiXmlElement* connsegidElement=new TiXmlElement("conn_segID");   
				 lineElement->LinkEndChild(connsegidElement);
				 connsegidElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._segline[i].conninfo[j].segid).c_str()));
				 //connection lineID
				 TiXmlElement* connlineidElement=new TiXmlElement("conn_lineID");       
				 lineElement->LinkEndChild(connlineidElement);
				 connlineidElement->LinkEndChild(new TiXmlText(NumTostring(all_seg->_roadsegment[seg_index]._segline[i].conninfo[j].lineid).c_str()));
			 }
         }

         seg_index++;
         RootElement->LinkEndChild(segmentElement);
     }

}

bool analysisStringBystrok(char *input , vector<uint32> &output)
{
    if(NULL == input)
    {
        return false;
	}
	
    output.clear();
    
    char * pch;
    pch = strtok (input," ,");

	int i = 0;
	uint32 tmp = 0;
    while (pch != NULL)
    {
        string str = pch;
        istringstream iss(str);
        iss >> tmp;
		if(tmp!=0)
		{output.push_back(tmp);}
        pch = strtok (NULL, " ,");
    }

    return true;
}

bool analysisString(char *input , vector<uint32> &output)
{
    if(NULL == input)
    {
        return false;
	}
	
    output.clear();
    int i=0;
    while(input[i]!='\0')
    {
        output.push_back(input[i]-48);
        i++;
    }

    return true;
}
int AnalyCfgFile(All_RoadSegment *p_all_seg,char *name)
{	
	FILE *fp = fopen(name,"r"); 
	if(NULL == fp)
	{
		return 0;
	}

    char ch[50];
	fscanf(fp,"StartSegmentID:%d\n",&(p_all_seg->segmentstartId));
	fscanf(fp,"segmentnum:%d\n",&(p_all_seg->segmentnum));
	
	fscanf(fp,"PreSegmentID:%s\n",ch);
	analysisStringBystrok(ch,PreSegmentID);

	fscanf(fp,"NextSegmentID:%s\n",ch);
    analysisStringBystrok(ch,NextSegmentID);

	fscanf(fp,"PlaceMark:%s\n",ch);
    analysisStringBystrok(ch,PlaceMark);
	
	fscanf(fp,"lane_num:%d\n",&(p_all_seg->laneNum));
	fscanf(fp,"line_num:%d\n",&(p_all_seg->lineNum));

	fscanf(fp,"LineTypeIndex:%s\n,",ch);
    analysisString(ch,LineTypeIndex);

 	fscanf(fp,"LaneDrectionIndex:%s\n,",ch);
    analysisString(ch,LaneDrectionIndex);

	fscanf(fp,"LaneAssign:%s\n",ch);
    analysisString(ch,LaneAssign);

    //conn info
    int index = 0;
    //while(index < NextSegmentID.size())
    while(!feof(fp))
    {
        conn_info_t conn_tmp;
        fscanf(fp,"LineConn:%d,%d,%d,%d\n",&conn_tmp.connlineS,&conn_tmp.connsegid
                                          ,&conn_tmp.connNextlineS,&conn_tmp.connlineNum);
        conn_info_g.push_back(conn_tmp);       
        index++;
    }
 
    fclose(fp);
	
	return 1;
}

bool getFileList(filename_node *root)
{
    if(NULL == root)
    {
        return false;
	}
	
    /**get fies list**/
    FILE* fp = fopen("./cfg/FileIndex.txt","r");
	if(NULL == fp)
	{
	    cout<<"can not open cfg!"<<endl;
		return false;
	}
	char ch[MAX_FILENAME_LENTH];
	fscanf(fp,"%s\n",ch);
	strcpy(root->name ,ch);

    while(!feof(fp))
    {
		fscanf(fp,"%s\n",ch);
        filename_node *next = new filename_node;
		strcpy(next->name ,ch);
		root->p_next = next;
		root = root->p_next;
	}
	
	root->p_next = NULL;
    fclose(fp); 
	
	return true;
}

void InitSegment(All_RoadSegment *p_all_seg)
{
    //p_all_seg->segType = 0;   
	p_all_seg->pointNum = 2;
    p_all_seg->_roadsegment.clear();	

	p_all_seg->laneNum = 0;
	p_all_seg->lineNum = 0;
	p_all_seg->segmentnum = 0;
    p_all_seg->segmentstartId = 0;

	PreSegmentID.clear();
	NextSegmentID.clear();
    LineTypeIndex.clear();
    LaneDrectionIndex.clear();
    LaneAssign.clear();
    conn_info_g.clear();
  	
    return;
}
int main(int argc, char* argv[])
{
    All_RoadSegment all_seg;
	int result;
    uint32 seg_type,lane_num,line_num,point_num;

    /*input the cfg files*/
    filename_node *root = new filename_node;
    getFileList(root);
	filename_node *pfile = root;

	/**generate the XML root **/
	TiXmlDocument *myDocument = new TiXmlDocument();
	TiXmlElement *RootElement = new TiXmlElement("All_segment");
	myDocument->LinkEndChild(RootElement);
	
	/*generate REF GPS*/
	TiXmlElement *reflatElement = new TiXmlElement("ref_point_lat");
    reflatElement->LinkEndChild(new TiXmlText(REFERENCE_LAT));	
	RootElement->LinkEndChild(reflatElement);
	TiXmlElement *reflonElement = new TiXmlElement("ref_point_lon");
    reflonElement->LinkEndChild(new TiXmlText(REFERENCE_LON));		
	RootElement->LinkEndChild(reflonElement);
	
	/**generate segment num **/
	TiXmlElement *segnumElement = new TiXmlElement("segmentnum");
    RootElement->LinkEndChild(segnumElement);  
	
    /**generate segment **/     
    int segIndex = 0;
	while(pfile!=NULL)
    {
        /**init**/
		InitSegment(&all_seg);
		
        /**get segment info**/
		result = AnalyCfgFile(&all_seg,pfile->name);
		if(!result)
        {
            cout<<"can not open cfg!"<<endl;
            return 0;
        }

		if(123 == all_seg.segmentstartId)
        {
        all_seg.segmentstartId = 123;
        }
				
        /*cfg segment*/
        CfgSegments(&all_seg);
        
        /**get gps**/
        result = GetGps(&all_seg);
        if(!result)
        {
            cout<<"get gps failed!"<<endl;
            return 0;
        }
	    /**generate the XML other info**/
        segIndex += all_seg.segmentnum;
        XML_genarate(RootElement , &all_seg);      
		
    	pfile = pfile->p_next;
    }

    segnumElement->LinkEndChild(new TiXmlText(NumTostring(segIndex).c_str()));

	/**save MyXML**/
    myDocument->SaveFile("out.xml");
	
    return 0;
}
