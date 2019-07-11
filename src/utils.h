inline bool ends_with(std::string value, std::string ending)
{
	size_t endingPos = value.rfind(ending);
	return endingPos != -1 && endingPos == (value.length() - ending.length());
}

inline void erase_last(std::string& base, std::string token)
{
	size_t tokenPos = base.rfind(token);
	
	if(tokenPos != -1)
	{
		base.erase(tokenPos, token.length());
	}
}
