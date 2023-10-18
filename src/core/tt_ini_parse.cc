#include"tt_ini_parse.h"

IniParse::IniParse(const std::string& path){
	read_ini(path);
}

void IniParse::read_ini(std::string path)
{
	std::ifstream in_conf_file(path.c_str());
	assert(in_conf_file && "open ini file failed");

	std::string str_line = "";
	std::string str_root = "";
	std::vector<IniNode> vec_ini;
	while (getline(in_conf_file, str_line))
	{
		if(str_line[0] == ';')
			continue;
		
		std::string::size_type left_pos = 0;
		std::string::size_type right_pos = 0;
		std::string::size_type equel_div_pos = 0;
		std::string str_key = "";
		std::string str_value = "";
		if ((str_line.npos != (left_pos = str_line.find("["))) && (str_line.npos != (right_pos = str_line.find("]"))))
			str_root = str_line.substr(left_pos+1, right_pos-1);

		if (str_line.npos != (equel_div_pos = str_line.find("="))){
			str_key = str_line.substr(0, equel_div_pos);
			str_value = str_line.substr(equel_div_pos+1, str_line.size()-1);
			str_key = trim_str(str_key);
			str_value = trim_str(str_value);
		}

		if((!str_root.empty()) && (!str_key.empty()) && (!str_value.empty())){
			IniNode ini_node(str_root, str_key, str_value);
			vec_ini.push_back(ini_node);
		}
	}

	in_conf_file.close();
	in_conf_file.clear();

	std::map<std::string, std::string> map_tmp;
	for(std::vector<IniNode>::iterator itr = vec_ini.begin(); itr != vec_ini.end(); ++itr)
		map_tmp.insert(std::pair<std::string, std::string>(itr->root, ""));

	SubNode sn;
	for(std::map<std::string, std::string>::iterator itr = map_tmp.begin(); itr != map_tmp.end(); ++itr){
		for(std::vector<IniNode>::iterator sub_itr = vec_ini.begin(); sub_itr != vec_ini.end(); ++sub_itr)
			if(sub_itr->root == itr->first) sn.insert_element(sub_itr->key, sub_itr->value);
		m_map_ini.insert(std::pair<std::string, SubNode>(itr->first, sn));
	}
}

std::string IniParse::get_str(std::string root, std::string key)
{
	std::map<std::string, SubNode>::iterator itr = m_map_ini.find(root);
	if(itr == m_map_ini.end())
		return m_non_value_return;
	
	std::map<std::string, std::string>::iterator sub_itr = itr->second.sub_node.find(key);
	if(sub_itr == itr->second.sub_node.end())
		return m_non_value_return;
	
	return sub_itr->second;
}

void IniParse::clear() { m_map_ini.clear(); }

std::string& IniParse::trim_str(std::string& str)
{
	std::string::size_type pos = 0;
	while(str.npos != (pos = str.find(" "))) str = str.replace(pos, pos+1, "");
	return str;
}



