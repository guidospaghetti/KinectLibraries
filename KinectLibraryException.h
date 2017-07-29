#include <exception>

class KinectLibraryException : public std::exception {

public:

	KinectLibraryException(const char* error) {
		_error = error;
	}

	virtual const char* what() const throw() {
		return _error;
	}

private:
	const char* _error;
};