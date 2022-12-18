#ifndef CRH_2022_PERSISTENT_COUNTER_HPP_
#define CRH_2022_PERSISTENT_COUNTER_HPP_

#include<fstream>
#include<filesystem>

class PersistentCounter
{
	int value;
	std::filesystem::path filename;
public:
	PersistentCounter(const std::string &_filename)
	: filename(_filename)
	{
		try {
			value = load_integer(filename);
		} catch(...) {
			hard_set(0);
		}
	}
	void hard_set(int _value)
	{
		value = _value;
		std::ofstream ofs(filename);
		ofs << value << std::endl;
		ofs.close();
	}
	int get() const
	{
		return value;
	}
	void set(int _value)
	{
		if(_value != value) {
			hard_set(_value);
		}
	}
	int operator++()
	{
		hard_set(value + 1);
		return value;
	}
	std::filesystem::file_time_type get_last_write_time() const
	{
		return std::filesystem::last_write_time(filename);
	}
	static int load_integer(const std::filesystem::path &filename)
	{
		std::ifstream ifs(filename);
		if(!ifs.is_open()) {
			throw std::runtime_error("PersistentCounter: file not found");
		}
		int res = 0;
		ifs >> res;
		if(!ifs) {
			throw std::runtime_error("PersistentCounter: read failed");
		}
		return res;
	}
};

#endif /* CRH_2022_PERSISTENT_COUNTER_HPP_ */