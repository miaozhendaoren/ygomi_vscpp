#include <sstream>
#include "XMLParser.h"

std::ostream& operator<<(std::ostream& os, TagElement& tag){
	const std::vector<Attribute> attrs = tag.getAllAttrs();

	return os<<"tag name:"<<tag.getName()<<" attrs:"<<attrs.size();
}
/*
static bool isFindAttr(Attribute p, std::string name){
	return p.first == name;
}
*/
static void split(const std::string str, const std::string del, std::vector<std::string>* result){
	if(NULL==result){
		return;
	}

	char* p = strtok(const_cast<char*>(str.c_str()), del.c_str());
	
	while(p!=NULL){
		//std::cout<<p<<std::endl;
		result->push_back(p);
		p=strtok(NULL, del.c_str());
	}
}

static Attribute string2Pair(const std::string str){
	std::vector<std::string> keyValueVector;
	split(str, std::string(TagElement::NAME_VALUE_SPLITER), &keyValueVector);

	Attribute p(keyValueVector.at(0), keyValueVector.at(1));
	return p;
}

static TagElement string2Tag(const std::string str){
	std::vector<std::string> tagSegVector;
	split(str, std::string(TagElement::TAG_ATTR_SPLITER), &tagSegVector);

	TagElement te;
	if(tagSegVector.size() == 1){
		te.setName(tagSegVector.at(0));
	}else if(tagSegVector.size() > 0){
		te.setName(tagSegVector.at(0));
		std::string attrString = tagSegVector.at(1);
		//std::cout<<attrString<<std::endl;
		std::vector<std::string> attrStrVector;
		split(attrString, std::string(TagElement::ATTR_SPLITER), &attrStrVector);

		int attrSize = attrStrVector.size();
		std::vector<Attribute> attrsVector(attrSize);

		transform(attrStrVector.begin(), attrStrVector.end(), attrsVector.begin(), string2Pair);

		te.setAllAttr(attrsVector);
	}
	//std::cout<<te<<std::endl;
	return te;
}

/******************** TagElement ***************************/
std::string TagElement::SEQ_SPLITER = "/";
std::string TagElement::TAG_ATTR_SPLITER = ":";
std::string TagElement::ATTR_SPLITER = "&";
std::string TagElement::NAME_VALUE_SPLITER = "=";

void TagElement::setName(const std::string name){
	this->tagName = name;
}

void TagElement::setAllAttr(std::vector<Attribute> atts){
	this->attrs = atts;
}

void TagElement::addAttr(const std::string name, const std::string value){
	this->attrs.push_back(Attribute(name, value));
}

std::string TagElement::getName() const{
	return this->tagName;
}

const std::vector<Attribute>& TagElement::getAllAttrs() const{
	return this->attrs;
}

std::string TagElement::getAttr(std::string name) const{
	const std::vector<Attribute> attrs = this->getAllAttrs();
	for(size_t i=0;i<attrs.size();i++){
		Attribute a = attrs.at(i);

		if(a.first == name){
			return a.second;
		}
	}

	return "";

	//std::vector<Attribute>::iterator it = std::find_if(attrs.begin(), attrs.end(), std::bind2nd(ptr_fun(isFindAttr), name));
	//return it == attrs.end()?std::string(""):it->second;
}

/******************** XMLParser ***************************/
XMLParser::XMLParser(const std::string file){
	this->fileName = file;
	this->doc = new TiXmlDocument(this->fileName.c_str());
	this->isLoaded = this->doc->LoadFile();
}

XMLParser::~XMLParser(){
	delete this->doc;
}

bool XMLParser::isLoad() const{
	return this->isLoaded;
}

void XMLParser::translate(const std::string str, std::vector<TagElement>* result){
	using std::vector;
	using std::string;

	std::vector<std::string> segVector;
	split(str, string(TagElement::SEQ_SPLITER), &segVector);

	size_t segSize = segVector.size();

	result->resize(segSize);

	if(segSize>0){
		transform(segVector.begin(), segVector.end(), result->begin(), string2Tag);
	}
}

bool XMLParser::isItemQualify(const TiXmlNode* item,const TagElement& tag){
	using std::string;
	string itemName(item->Value());

	bool isOK = false;
	if(itemName==tag.getName()){
		std::vector<Attribute> attrs = tag.getAllAttrs();
		const TiXmlElement* el = item->ToElement();
		isOK = true;

		for(std::vector<Attribute>::iterator it = attrs.begin(); it != attrs.end(); it++){
			const char* att = el->Attribute(it->first.c_str());
			if(NULL==att || it->second != string(att)){
				isOK = false;
				break;
			}
		}
	}

	return isOK;
}

void XMLParser::getValuesByPath(std::vector<std::string>* result,const TiXmlNode* startItem, 
		const std::vector<TagElement> pathVector, const int i=0){
	using std::vector;
	using std::string;

	TagElement el = pathVector.at(i);
	if(this->isItemQualify(startItem, el)){
		if(i==pathVector.size()-1){//the last one in path
			const TiXmlElement* el = startItem->ToElement();
			const char* text = el->GetText();

			if(NULL!=text){
				result->push_back(el->GetText());
			}
		}else{
			for(const TiXmlNode* item = startItem->FirstChild(); NULL!=item; item = item->NextSibling()){
				this->getValuesByPath(result, item, pathVector, i+1);
			}
		}
	}
}

int XMLParser::getValues(const std::string path, std::vector<std::string>* values){
	using std::vector;
	using std::string;

	if(path.empty() || NULL==values){//path is illegal
		return -1;
	}

	vector<TagElement> pathVector;
	translate(string(path), &pathVector);

	if(pathVector.size()<1){
		return -2;
	}

	if(this->fileName.empty()){
		return -3;// file name is null
	}

	if(this->isLoad()){
		//std::cout<<"------load successfull------"<<std::endl; 
		TiXmlElement* root = this->doc->RootElement();
		
		if(NULL==root){
			return -4;
		}else{
			this->getValuesByPath(values, root, pathVector);
			return 1;
		}
	}else{
		return -5; //load fail
	}
}
