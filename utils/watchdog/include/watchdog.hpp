#ifndef CRH_2022_WATCHDOG_HPP_
#define CRH_2022_WATCHDOG_HPP_

#include <type_traits>
#include <vector>

#include <data.hpp>
#include <umt.hpp>
#include <log.hpp>
#include <persistent_counter.hpp>

struct has_reset_impl {
	template<class T>
	static auto check(T*) -> std::is_same<decltype(&T::Reset), bool(T::*)()>;
	static auto check(...) -> std::false_type;
};
/// to check if a type provides a reset interface
template<class T>
struct has_reset : decltype(has_reset_impl::check(std::declval<T*>())) {};

struct is_reset_impl {
	template<class T>
	static auto check(T*) -> std::is_same<T, bool()>;
	static auto check(...) -> std::false_type;
};
/// to check if a type itself is a reset interface
template<class T>
struct is_reset : decltype(is_reset_impl::check(std::declval<T*>())) {};

class ResetHandlerBase
{
protected:
	ResetHandlerBase() = default;
public:
	virtual bool reset() {
		return false;
	}
};

template<class T>
class ResetHandler : public ResetHandlerBase
{
	T &t;
public:
	ResetHandler(T &t)
	: ResetHandlerBase(), t(t)
	{ }
	bool reset() override {
		if constexpr (has_reset<T>::value) {
			return this->t.Reset();
		} else if constexpr (is_reset<T>::value) {
			return this->t();
		} else {
			return false;
		}
	}
};

class WatchDog
{
	std::vector<std::string> topics; // e.g.: for module [Predictor], the topic string is "Predictor", and then the topic string in [umt] is "Predictor_HB"
	std::vector<std::shared_ptr<umt::Subscriber<HEART_BEAT>>> subs;
	std::vector<std::shared_ptr<ResetHandlerBase>> handlers;

	Logger logger;
	std::thread thread;

	PersistentCounter cnt; // store the how many times we remaked
private:
	void remake()
	{
		using namespace std::chrono_literals;
		std::chrono::time_point last_write = cnt.get_last_write_time();
		if(auto delta_t = std::filesystem::__file_clock::now() - last_write; delta_t > 1min) { // it has been a long time since the last remake, let us reset the counter
			logger.info("Counter expired ({} seconds ago), resetting...", std::chrono::duration_cast<std::chrono::seconds>(delta_t).count());
			cnt.set(0);
		}

		if(cnt.get() == 3) { // if we have remaked 3 times, then we reboot the system
			logger.critical("Remake counter reached 3, rebooting...");
			// std::system("reboot"); // TODO: enable this
			param.set_run_mode(MODE::HALT);
			return;
		}
		logger.warn("Remaking: try {}", ++cnt);
		param.set_run_mode(MODE::HALT);
	}
public:
	WatchDog(const std::string &cnt_file)
	: logger("WatchDog"), cnt(cnt_file)
	{ }

	template<class T>
	void Watch(const std::string &topic, T &&module) {
		topics.push_back(topic);
		subs.push_back(std::make_shared<umt::Subscriber<HEART_BEAT>>(topic + "_HB"));
		handlers.push_back(std::make_shared<ResetHandler<std::remove_reference_t<T>>>(module));
	}
	void Run(std::chrono::seconds delay, std::chrono::seconds interval) {
		thread = std::thread([this, delay=delay, interval=interval]() {
			std::this_thread::sleep_for(delay);
			std::pair<bool, HEART_BEAT> msg;
			auto temp = std::chrono::system_clock::now().time_since_epoch() - cnt.get_last_write_time().time_since_epoch();
			while (param.get_run_mode() != MODE::HALT) {
				for (size_t i = 0; i < topics.size(); i++) {
					try {
						msg = subs[i]->try_pop();
					} catch(const HaltEvent&) {
						return;
					}
					if(!msg.first) {
						logger.warn("{} is not alive, trying to reset...", topics[i]);
						if(handlers[i]->reset()) {
							logger.info("{} is reset successfully", topics[i]);
						} else {
							logger.error("Reset {} failed", topics[i]);
							remake();
							break;
						}
					}
				}
				std::this_thread::sleep_for(interval);
			}
		});
	}
	void Join() {
		logger.info("Waiting for WatchDog to stop");
		if(thread.joinable()) {
			thread.join();
		}
		logger.sinfo("[WatchDog] joined.");
	}
};

#endif /* CRH_2022_WATCHDOG_HPP_ */