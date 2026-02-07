#pragma once

#include <vislib.hpp>

namespace vislib {

template <typename Port_t> using CallbackPortInitializer = core::Callable<core::Error, Port_t>;

using CallbackFunctor = core::Callable<void>;

template<typename Port_t> struct CallbackBase {
    CallbackFunctor functor;
    Port_t port;

    void execute() {
        functor.execute();
    }

    void operator()() {
        functor.execute();
    }

    bool isValid() const {
        return functor.isValid();
    }
};

template<typename Port_t> using CallbackChecker = core::Callable<bool, CallbackBase<Port_t>>;
template<typename Port_t> using CallbackInitializer = core::Callable<core::Error, CallbackBase<Port_t>>;
template<typename Port_t> using CallbackAttacher = core::Callable<core::Error, CallbackBase<Port_t>>;

template<typename Port_t> using CallbackExecutor = core::Callable<core::Error, CallbackBase<Port_t>>;
using CallbackErrorHandler = core::Callable<core::Error, core::Error>;

template<typename Port_t> static CallbackExecutor<Port_t> defaultCallbackExecutor = [](CallbackBase<Port_t>& callback) -> core::Error {
    if (!callback.isValid()) return {core::ErrorCode::invalidResource, "Cannot call a Null callable object"};
    callback.execute();
    return {};
};

static CallbackErrorHandler defaultErrorHandler = [](const core::Error& error) -> core::Error {
    return error;
};

template <typename Port_t> class CallbackInterface {
public:
    virtual core::Error initialize() = 0;
    virtual core::Error attach() = 0;
    virtual bool check() const = 0;
    virtual core::Error execute() = 0;
    virtual core::Error operator()() = 0;
    virtual bool isValid() const = 0;
    virtual Port_t port() const = 0;

    virtual ~CallbackInterface() = default;
};

template <typename Port_t> class Callback : virtual public CallbackInterface<Port_t> {
protected:
    CallbackInterface<Port_t>* implementation = nullptr;
public:
    Callback() = default;

    template <typename T> Callback(T implementation) {
        this->implementation = static_cast<CallbackInterface<Port_t>*>(new T(implementation));
    }

    core::Error initialize() override {
        return implementation->initialize();
    }

    core::Error attach() override {
        return implementation->attach();
    }

    bool check() const override {
        return implementation->check();
    }

    core::Error execute() override {
        return implementation->execute();
    }

    core::Error operator()() override {
        return implementation->operator()();
    }

    bool isValid() const override {
        return implementation->isValid();
    }

    Port_t port() const override {
        return implementation->port();
    }

    ~Callback() override = default;
};

template<typename Port_t> class CallbackSingle : virtual public CallbackInterface<Port_t> {
protected:

    CallbackBase<Port_t> callback;
    CallbackInitializer<Port_t> initializer;
    CallbackAttacher<Port_t> attacher;
    CallbackChecker<Port_t> checker;
    CallbackExecutor<Port_t> executor;
    CallbackErrorHandler errorHandler;

public:
    CallbackSingle(
        const CallbackBase<Port_t>& callback,
        const CallbackInitializer<Port_t>& initializer,
        const CallbackAttacher<Port_t>& attacher,
        const CallbackChecker<Port_t>& checker,
        const CallbackExecutor<Port_t>& executor = defaultCallbackExecutor<Port_t>,
        const CallbackErrorHandler& errorHandler = defaultErrorHandler) {

        this->callback = callback;
        this->initializer = initializer;
        this->attacher = attacher;
        this->checker = checker;
        this->executor = executor;
        this->errorHandler = errorHandler;
    }

    core::Error initialize() override {
        return initializer.execute(callback);
    }

    core::Error attach() override {
        return attacher.execute(callback);
    }

    bool check() const override {
        return static_cast<CallbackChecker<Port_t>>(checker).execute(callback);
    }

    core::Error execute() override {

        if (!checker.execute(callback)) return {};

        core::Error error = executor.execute(callback);

        error = errorHandler.execute(error);

        if (error) return error;

        return {};
    }

    core::Error operator()() override {
        return this->execute();
    }

    bool isValid() const override {
        return callback.isValid();
    }

    Port_t port() const override {
        return callback.port;
    }

    ~CallbackSingle() override = default;
};

template<typename Port_t> class CallbackFamily : virtual public CallbackInterface<Port_t> {
protected:
    Port_t port_;

    core::Array<CallbackBase<Port_t>> family;

    CallbackInitializer<Port_t> initializer;
    CallbackAttacher<Port_t> attacher;
    CallbackChecker<Port_t> checker;
    CallbackExecutor<Port_t> executor;
    CallbackErrorHandler errorHandler;

public:
    CallbackFamily(
        const Port_t& port_,
        const core::Array<CallbackBase<Port_t>>& group,
        const CallbackInitializer<Port_t>& initializer,
        const CallbackAttacher<Port_t>& attacher,
        const CallbackChecker<Port_t>& checker,
        const CallbackExecutor<Port_t>& executor = defaultCallbackExecutor<Port_t>,
        const CallbackErrorHandler& errorHandler = defaultErrorHandler) {

        this->port_ = port_;
        this->family = group;
        this->initializer = initializer;
        this->attacher = attacher;
        this->checker = checker;
        this->executor = executor;
        this->errorHandler = errorHandler;

    }


    core::Error initialize() override {
        core::Error error;
        for (size_t i = 0; i < family.Size(); i++) {
            error = initializer.execute(family[i]);

            if (error) return error;

        }

        return {};
    }

    core::Error attach() override {
        core::Error error;
        for (size_t i = 0; i < family.Size(); i++) {
            error = attacher.execute(family[i]);

            if (error) return error;
        }

        return {};
    }

    bool check() const override {
        for (size_t i = 0; i < family.Size(); i++)
            if (checker.execute(family[i])) return true;

        return false;

    }

    core::Error execute() override {
        core::Error error;
        for (size_t i = 0; i < family.Size(); i++) {
            if (!checker.execute(family[i])) continue;

            error = executor.execute(family[i]);
            error = errorHandler.execute(error);

            if (error) return error;
        }

        return {};
    }

    core::Error operator()() override {
        return this->execute();
    }

    bool isValid() const override {
        for (size_t i = 0; i < family.Size(); i++) {
            if (!family[i].isValid()) return false;
        }

        return true;
    }

    Port_t port() const override {
        return port_;
    }

    ~CallbackFamily() override = default;

};


template <typename Port_t> class CallbackTable {
protected:
    core::Array<Port_t> ports;
    core::Array<Callback<Port_t>> callbacks;

    bool initialized = false;

public:

    core::Error InitCallbackTable(const core::Array<Port_t>& ports, const CallbackPortInitializer<Port_t>& portInitializer) noexcept(core::numberNoexcept<Port_t>()) {

        core::Error error;

        for (size_t i = 0; i < ports.Size(); i++) {
            error = const_cast<CallbackPortInitializer<Port_t>&>(portInitializer).execute(ports[i]);

            if (error) return error;
        }

        this->ports = ports;


        Port_t max = ports[0];
        for (size_t i = 0; i < ports.Size(); i++) max = max >= ports[i] ? max : ports[i];


        callbacks = core::Array<Callback<Port_t>>(max + 1);

        initialized = true;

        return {};
    }

    constexpr bool isInitialized() const noexcept {
        return initialized;
    }

    bool isCallbackPort(const Port_t& port) const noexcept(core::comparableNoexcept<Port_t>()) {
        bool found = false;

        if (!initialized) return false;

        for (size_t i = 0; i < ports.Size(); i++) {
            if (ports[i] == port) {
                found = true;
                break;
            }
        }

        return found;
    }

    core::Error setCallback(const Callback<Port_t>& callback) noexcept(core::numberNoexcept<Port_t>()) {

        if (!initialized) return {core::ErrorCode::invalidConfiguration, "The callback table wasn't initialized"};

        if (!isCallbackPort(callback.port())) return
        {core::ErrorCode::invalidArgument, "The port " + core::to_string(static_cast<size_t>(callback.port())) + " is not set for callbacks"};

        callbacks[static_cast<size_t>(callback.port())] = callback;

        core::Error error = const_cast<Callback<Port_t>&>(callback).initialize();
        if (error) return error;

        error = const_cast<Callback<Port_t>&>(callback).attach();
        if (error) return error;

        return {};
    }

    core::Error manualProcess(const Port_t& port) noexcept(core::numberNoexcept<Port_t>()) {
        if (!initialized) return {core::ErrorCode::invalidConfiguration, "The callback table wasn't initialized"};

        if (!isCallbackPort(port)) return
        {core::ErrorCode::invalidArgument, "The port " + core::to_string(static_cast<size_t>(port)) + " is not set for callbacks"};

        if (!callbacks[static_cast<size_t>(port)].isValid()) return
        {core::ErrorCode::invalidConfiguration, "The port " + core::to_string(static_cast<size_t>(port)) + " has no set callback"};

        if (!callbacks[static_cast<size_t>(port)].check()) return {};

        core::Error e = callbacks[static_cast<size_t>(port)].execute();

        if(e) return e;

        return {};

    }

    core::Error manualProcess() noexcept(core::numberNoexcept<Port_t>()) {
        if (!initialized) return {core::ErrorCode::invalidConfiguration, "The callback table wasn't initialized"};

        core::Error error;

        for (size_t i = 0; i < ports.Size(); i++) {
            if (!callbacks[static_cast<size_t>(ports[i])].isValid() || !callbacks[static_cast<size_t>(ports[i])].check()) continue;
            error = callbacks[ports[i]].execute();
            if (error) return error;
        }
        return {};
    }
};

} // namespace vislib