#pragma once

// =============================================================================
// ModbusCallback - every handler the library stores
// =============================================================================
//
// A plain function and the pointer it is called with. That is all the library
// itself ever stores, so a queued request, a change handler or a register
// reader costs four words and no heap, and the firmware carries no
// std::function code.
//
//   static void onDone(void* context, ModbusResult result) { ... }
//   node.readHreg(17, 40, value, ModbusDone(onDone, &state));
//
// The library's own code also uses a third form, a function that receives a
// number as well as the context - a poll index, a request generation - so it
// never needs to allocate to remember one:
//
//   static void onPoll(void* poller, uintptr_t index, ModbusResult result);
//   ModbusDone(onPoll, this, index)
//
// Anything else that can be called with the right arguments is accepted as
// well: a function, a lambda with or without captures, a std::function. A
// captureless lambda or a function is stored as it is. Anything with state is
// copied into the heap, and copying the callback copies it again, exactly as a
// std::function does. That code is a template in this header, so it is
// compiled only into a program that passes one; the library never does.
//
//   node.readHreg(17, 40, value, [&](ModbusResult r) { done = true; });

#include <stddef.h>
#include <stdint.h>

#include <new>
#include <type_traits>
#include <utility>

// Template parameters are never called F: Arduino.h defines F() as a macro.
template <typename Signature>
class ModbusCallback;

template <typename R, typename... A>
class ModbusCallback<R(A...)> {
    // Whether T can stand in for R(A...). Hand-written rather than
    // std::is_invocable, which needs C++17; some Arduino cores are on C++14.
    template <typename T, typename = void>
    struct Callable : std::false_type {};
    template <typename T>
    struct Callable<T, typename std::enable_if<
        std::is_void<R>::value ||
        std::is_convertible<decltype(std::declval<T&>()(std::declval<A>()...)), R>::value
    >::type> : std::true_type {};

public:
    // The function forms.
    typedef R (*Function)(void* context, A... args);
    typedef R (*TaggedFunction)(void* context, uintptr_t tag, A... args);
    typedef R (*Plain)(A... args);

    ModbusCallback() noexcept {}
    ModbusCallback(std::nullptr_t) noexcept {}
    ModbusCallback(Function function, void* context)
        : _fn(function ? &callFunction : nullptr), _ctx(context),
          _tag(reinterpret_cast<uintptr_t>(function)) {}
    ModbusCallback(TaggedFunction function, void* context, uintptr_t tag)
        : _fn(function), _ctx(context), _tag(tag) {}

    // Any other callable. Constrained to what can be called with these
    // arguments, so overloads taking different handler types stay distinct.
    template <typename T,
              typename D = typename std::decay<T>::type,
              typename = typename std::enable_if<
                  !std::is_same<D, ModbusCallback>::value &&
                  !std::is_same<D, std::nullptr_t>::value &&
                  !std::is_same<D, Function>::value &&
                  !std::is_same<D, TaggedFunction>::value &&
                  Callable<D>::value>::type>
    ModbusCallback(T&& f) {
        adopt(std::forward<T>(f), std::is_convertible<D, Plain>());
    }

    ModbusCallback(const ModbusCallback& other)
        : _fn(other._fn), _ctx(other._ctx), _tag(other._tag), _box(other._box) {
        if (_box) _ctx = _box->clone(other._ctx);
    }
    ModbusCallback(ModbusCallback&& other) noexcept
        : _fn(other._fn), _ctx(other._ctx), _tag(other._tag), _box(other._box) {
        other._fn = nullptr;
        other._box = nullptr;
    }
    ModbusCallback& operator=(ModbusCallback other) noexcept {
        swap(other);
        return *this;
    }
    ~ModbusCallback() {
        if (_box) _box->destroy(_ctx);
    }

    explicit operator bool() const { return _fn != nullptr; }
    friend bool operator==(const ModbusCallback& c, std::nullptr_t) { return !c._fn; }
    friend bool operator!=(const ModbusCallback& c, std::nullptr_t) { return c._fn != nullptr; }

    R operator()(A... args) const { return _fn(_ctx, _tag, args...); }

    void swap(ModbusCallback& other) noexcept {
        std::swap(_fn, other._fn);
        std::swap(_ctx, other._ctx);
        std::swap(_tag, other._tag);
        std::swap(_box, other._box);
    }

private:
    struct Box {
        void* (*clone)(const void*);
        void (*destroy)(void*);
    };

    template <typename T>
    struct Boxed {
        static R call(void* c, uintptr_t, A... a) { return (*static_cast<T*>(c))(a...); }
        static void* clone(const void* c) { return new T(*static_cast<const T*>(c)); }
        static void destroy(void* c) { delete static_cast<T*>(c); }
        static const Box* box() {
            static const Box table = { &clone, &destroy };   // constant: no guard
            return &table;
        }
    };

    static R callFunction(void* c, uintptr_t fn, A... a) {
        return reinterpret_cast<Function>(fn)(c, a...);
    }
    static R callPlain(void*, uintptr_t fn, A... a) {
        return reinterpret_cast<Plain>(fn)(a...);
    }

    // A function, or a lambda without captures: stored as a pointer.
    template <typename T>
    void adopt(T&& f, std::true_type) {
        const Plain plain = f;
        if (!plain) return;
        _fn = &callPlain;
        _tag = reinterpret_cast<uintptr_t>(plain);
    }

    // Anything with state: copied into the heap. An empty std::function stays
    // an empty callback rather than one that throws when called.
    template <typename T>
    void adopt(T&& f, std::false_type) {
        typedef typename std::decay<T>::type D;
        if (isEmpty(f, 0)) return;
        _ctx = new D(std::forward<T>(f));
        _fn = &Boxed<D>::call;
        _box = Boxed<D>::box();
    }
    template <typename T>
    static auto isEmpty(const T& f, int) -> decltype(static_cast<bool>(f), bool()) {
        return !static_cast<bool>(f);
    }
    template <typename T>
    static bool isEmpty(const T&, long) { return false; }

    TaggedFunction _fn = nullptr;
    void* _ctx = nullptr;
    uintptr_t _tag = 0;
    const Box* _box = nullptr;
};
