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

template<typename T>
inline typename PointMatcher<T>::TransformationParameters matrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix, int dim)
{
	typedef typename PointMatcher<T>::TransformationParameters M;
	assert(matrix.rows() == matrix.cols());
	assert((matrix.rows() == 3) || (matrix.rows() == 4));
	assert((dim == 3) || (dim == 4));
	
	if(matrix.rows() == dim)
	{
		return matrix;
	}
	
	M out(M::Identity(dim, dim));
	out.topLeftCorner(2, 2) = matrix.topLeftCorner(2, 2);
	out.topRightCorner(2, 1) = matrix.topRightCorner(2, 1);
	return out;
}

template
PointMatcher<float>::TransformationParameters matrixToDim<float>(const PointMatcher<float>::TransformationParameters& matrix, int dim);

template
PointMatcher<double>::TransformationParameters matrixToDim<double>(const PointMatcher<double>::TransformationParameters& matrix, int dim);
