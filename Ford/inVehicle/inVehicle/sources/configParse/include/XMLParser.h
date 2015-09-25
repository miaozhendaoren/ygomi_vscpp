#include <iostream>
#include "tinyxml.h"
#include "tinystr.h"
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <iterator>
#include <sstream>

typedef std::pair<std::string, std::string> Attribute;

class TagElement{
	std::string tagName;
	std::vector<Attribute> attrs;
public:
	static std::string SEQ_SPLITER;
	static std::string TAG_ATTR_SPLITER;
	static std::string ATTR_SPLITER;
	static std::string NAME_VALUE_SPLITER;

	void setName(const std::string name);
	void setAllAttr(const std::vector<Attribute> atts);
	void addAttr(const std::string name, const std::string value);

	std::string getName() const;
	const std::vector<Attribute>& getAllAttrs() const;
	std::string getAttr(std::string name) const;

	friend std::ostream& operator<<(std::ostream& os, TagElement& tag);
};

class XMLParser{
	std::string fileName;
	TiXmlDocument* doc;
	bool isLoaded;
	void translate(const std::string  str, std::vector<TagElement>* result);
	bool isItemQualify(const TiXmlNode* item, const TagElement& tag);
	void getValuesByPath(std::vector<std::string>* result, const TiXmlNode* startItem, const std::vector<TagElement> pathVector, const int i);

public:
	XMLParser(const std::string file);
	virtual ~XMLParser();

	bool isLoad() const;
	int getValues(const std::string path, std::vector<std::string>* values);
};


