The steps of generating the .XML

1. according to google map from "MyPlaces.kml" divide the segments and generate 
   "input.txt" by hand

2. copy "input.txt" into .\gen_XML_cfg\gen_XML_cfg\gen_XML_cfg\cfg
   run gen_XML_cfg , generate "FileIndex.txt" && "cfg_segmentXX.txt"
  
3. modify cfg_segmentXX.txt according to google map from "MyPlaces.kml" by hand

4. copy "FileIndex.txt" && "cfg_segmentXX.txt" into .\gen_segment_XML\gen_segment_xml\gen_segment_xml\cfg
   run gen_segment_xml generate the "out.xml"
   
5. modify "out.xml" by hand according to google map from "MyPlaces.kml"   