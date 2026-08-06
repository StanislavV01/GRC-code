#ifndef BACKTRACK_TEST_HARNESS_HPP
#define BACKTRACK_TEST_HARNESS_HPP

// Minimal zero-dependency test harness. No GoogleTest / package manager needed,
// in keeping with the TZ's offline/on-board constraint. Register cases with
// TEST(name); assert with the CHECK_* macros.

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace testh {

struct Case {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back(Case{name, std::move(fn)});
    }
};

struct Failure {
    std::string message;
};

inline void fail(const std::string& expr, const std::string& file, int line,
                 const std::string& extra) {
    throw Failure{file + ":" + std::to_string(line) + "  " + expr +
                  (extra.empty() ? "" : "  (" + extra + ")")};
}

inline int run_all() {
    int passed = 0;
    int failed = 0;
    for (const Case& c : registry()) {
        try {
            c.fn();
            std::cout << "  ok   " << c.name << '\n';
            ++passed;
        } catch (const Failure& f) {
            std::cout << "  FAIL " << c.name << "\n       " << f.message << '\n';
            ++failed;
        } catch (const std::exception& e) {
            std::cout << "  FAIL " << c.name << "  threw: " << e.what() << '\n';
            ++failed;
        }
    }
    std::cout << "\n" << passed << " passed, " << failed << " failed, "
              << registry().size() << " total\n";
    return failed == 0 ? 0 : 1;
}

}  // namespace testh

#define TEST(name)                                                        \
    static void name();                                                   \
    static ::testh::Registrar registrar_##name(#name, name);             \
    static void name()

#define CHECK_TRUE(cond)                                                   \
    do {                                                                   \
        if (!(cond)) ::testh::fail(#cond, __FILE__, __LINE__, "");        \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                              \
    do {                                                                   \
        const double va = (a);                                            \
        const double vb = (b);                                            \
        if (std::fabs(va - vb) > (tol))                                   \
            ::testh::fail(#a " ~= " #b, __FILE__, __LINE__,              \
                          std::to_string(va) + " vs " + std::to_string(vb)); \
    } while (0)

#define CHECK_LE(a, b)                                                     \
    do {                                                                   \
        const double va = (a);                                            \
        const double vb = (b);                                            \
        if (!(va <= vb))                                                   \
            ::testh::fail(#a " <= " #b, __FILE__, __LINE__,              \
                          std::to_string(va) + " vs " + std::to_string(vb)); \
    } while (0)

#define CHECK_LT(a, b)                                                     \
    do {                                                                   \
        const double va = (a);                                            \
        const double vb = (b);                                            \
        if (!(va < vb))                                                    \
            ::testh::fail(#a " < " #b, __FILE__, __LINE__,               \
                          std::to_string(va) + " vs " + std::to_string(vb)); \
    } while (0)

#define CHECK_GE(a, b)                                                     \
    do {                                                                   \
        const double va = (a);                                            \
        const double vb = (b);                                            \
        if (!(va >= vb))                                                   \
            ::testh::fail(#a " >= " #b, __FILE__, __LINE__,              \
                          std::to_string(va) + " vs " + std::to_string(vb)); \
    } while (0)

#define CHECK_GT(a, b)                                                     \
    do {                                                                   \
        const double va = (a);                                            \
        const double vb = (b);                                            \
        if (!(va > vb))                                                    \
            ::testh::fail(#a " > " #b, __FILE__, __LINE__,               \
                          std::to_string(va) + " vs " + std::to_string(vb)); \
    } while (0)

#endif  // BACKTRACK_TEST_HARNESS_HPP
