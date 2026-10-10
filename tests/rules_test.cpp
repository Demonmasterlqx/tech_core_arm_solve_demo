#include "tech_core/rules.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
constexpr double pi = 3.14159265358979323846;
using tech_core::AssemblyRules;
using tech_core::Observation;
using tech_core::Stage;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Callable> void require_throws(Callable callable, const char* message) {
    bool threw = false;
    try { callable(); } catch (const std::exception&) { threw = true; }
    require(threw, message);
}

struct Attempt {
    AssemblyRules rules;
    std::array<Observation, 2> obs{};
    explicit Attempt(int level, std::array<double, 2> targets = {0.5, -0.5})
        : rules(level, targets) { rules.update(0, obs); }
    void update(double time) { rules.update(time, obs); }
    void insert_both(double time) {
        for (auto& item : obs) item.insertion = 0.1;
        update(time);
    }
    void lift_both(double time) {
        for (auto& item : obs) item.lift = 0.1;
        update(time);
    }
    void flip_both(double time) {
        for (auto& item : obs) item.flip = pi / 2;
        update(time);
    }
    void align_both(double time) {
        obs[0].twist = 0.5;
        obs[1].twist = -0.5;
        update(time);
        update(time + 0.4);
    }
    void assemble() {
        insert_both(1);
        lift_both(2);
        flip_both(3);
        align_both(4);
    }
};

void test_lower_difficulties_and_cooldown() {
    Attempt one(1);
    one.insert_both(1);
    require(one.rules.confirm(0, 1), "level 1 needs only insertion");
    require(one.rules.successful(), "level 1 must finish");
    require(!one.rules.safety_locked(0), "level 1 has no Q safety lock");

    Attempt two(2);
    two.insert_both(1);
    require(!two.rules.confirm(0, 1), "level 2 must reject missing lift");
    require(two.rules.confirm_available_at(0) == 2, "rejected confirmation must block one second");
    two.lift_both(1.1);
    require(!two.rules.confirm(0, 1.9), "correct assembly cannot bypass cooldown");
    require(two.rules.confirm(0, 2), "confirmation allowed at exact cooldown endpoint");
    require(two.rules.successful(), "level 2 must finish without P or Q rotation");
}

void test_early_insertion_and_sequence() {
    AssemblyRules rules(3, {0.5, 0});
    std::array<Observation, 2> obs{};
    obs[0].station_ready = false;
    obs[0].insertion = 0.1;
    rules.update(0, obs);
    obs[0].station_ready = true;
    rules.update(1, obs);
    require(rules.insertion_blocked(0), "early insertion must not become valid on arrival");
    require(rules.stage(0) == Stage::Ready, "early insertion cannot complete step 2");
    obs[0].insertion = 0;
    rules.update(2, obs);
    require(!rules.insertion_blocked(0) && !rules.failed(), "withdraw early insertion to retry");
    obs[0].insertion = 0.1;
    obs[0].lift = 0.1;
    obs[0].flip = pi / 2;
    rules.update(3, obs);
    require(rules.stage(0) == Stage::Ready, "cannot skip physical sequence");
    obs[0].lift = 0;
    obs[0].flip = 0;
    rules.update(4, obs);
    require(rules.stage(0) == Stage::Inserted, "correct retry must insert");
}

void test_dwell_and_lock() {
    Attempt attempt(3);
    attempt.insert_both(1);
    attempt.lift_both(2);
    attempt.flip_both(3);
    attempt.obs[0].twist = 0.5 + 5 * pi / 180;
    attempt.update(4);
    attempt.update(4.399);
    require(attempt.rules.stage(0) == Stage::Flipped, "dwell cannot finish early");
    attempt.update(4.4);
    require(attempt.rules.stage(0) == Stage::Aligned, "5 degree boundary must be accepted");
    attempt.update(6);
    require(!attempt.rules.safety_locked(0), "lock requires strictly more than two seconds");
    attempt.update(6.001);
    require(attempt.rules.safety_locked(0), "continuous correct dwell must engage lock");
    attempt.obs[0].twist += 0.001;
    attempt.update(6.01);
    require(!attempt.rules.safety_locked(0), "angle drift must remove lock");
    require(attempt.rules.stage(0) == Stage::Flipped, "angle drift must clear alignment");
    attempt.obs[0].twist = 0.5;
    attempt.update(6.1);
    attempt.update(6.499);
    require(attempt.rules.stage(0) != Stage::Aligned, "reentry must repeat full dwell");
    attempt.update(6.5);
    require(attempt.rules.confirm(0, 6.5), "level 3 valid assembly confirms");
}

void test_all_prerequisites_remain_live() {
    for (int coordinate = 0; coordinate < 4; ++coordinate) {
        Attempt attempt(3);
        attempt.assemble();
        switch (coordinate) {
        case 0: attempt.obs[0].insertion = 0.09; break;
        case 1: attempt.obs[0].lift = 0.09; break;
        case 2: attempt.obs[0].flip = pi / 2 - 0.1; break;
        case 3: attempt.obs[0].twist = 0.7; break;
        }
        attempt.update(4.5);
        require(!attempt.rules.confirm(0, 4.5), "historical completion cannot hide current pose drift");
        require(!attempt.rules.successful(), "drifted assembly cannot succeed");
    }
    Attempt two(2);
    two.insert_both(1);
    two.lift_both(2);
    two.obs[0].lift = 0.08;
    two.update(3);
    require(!two.rules.confirm(0, 3), "level 2 must recheck lift at confirmation");
}

void test_repeated_confirmation_and_terminal_result() {
    Attempt attempt(3);
    attempt.insert_both(1);
    attempt.lift_both(2);
    attempt.flip_both(3);
    attempt.obs[0].twist = 0.5;
    attempt.update(4);
    require(!attempt.rules.confirm(0, 4), "confirmation cannot bypass angle dwell");
    require(!attempt.rules.confirm(0, 4.2), "repeated confirmation in cooldown ignored");
    require(attempt.rules.confirm_available_at(0) == 5, "ignored input must not extend cooldown");
    attempt.update(4.4);
    require(attempt.rules.stage(0) == Stage::Aligned, "confirmation failures must not reset valid dwell");
    require(!attempt.rules.confirm(0, 4.5), "alignment does not cancel confirmation cooldown");
    require(attempt.rules.confirm(0, 5), "confirmation succeeds at original cooldown endpoint");
    require(!attempt.rules.confirm(0, 5), "terminal confirmation cannot be awarded twice");
    // Automatic recovery removes the unit after success. The completed result
    // must remain terminal; withdrawal failure only applies during assembly.
    attempt.obs[0].insertion = 0;
    attempt.obs[0].robot_alive = false;
    attempt.update(6);
    require(attempt.rules.successful() && !attempt.rules.failed(), "recovery cannot revoke awarded success");
    require(attempt.rules.stage(0) == Stage::Confirmed, "successful stage must persist");
}

void test_twist_follows_flip() {
    Attempt attempt(3);
    attempt.obs[0].insertion = 0.1;
    attempt.obs[0].twist = 0.5;
    attempt.update(1);
    require(attempt.rules.stage(0) == Stage::Ready, "early Q target cannot bypass insertion sequence gate");
    attempt.obs[0].twist = 0;
    attempt.update(1.1);
    require(attempt.rules.stage(0) == Stage::Inserted, "neutral Q permits insertion");

    attempt.obs[0].lift = 0.1;
    attempt.obs[0].twist = 0.5;
    attempt.update(2);
    require(attempt.rules.stage(0) == Stage::Inserted, "early Q target cannot bypass lift sequence gate");
    attempt.obs[0].twist = 0;
    attempt.update(2.1);
    require(attempt.rules.stage(0) == Stage::Lifted, "neutral Q permits lift completion");

    attempt.obs[0].flip = pi / 2;
    attempt.obs[0].twist = 0.5;
    attempt.update(3);
    attempt.update(3.5);
    require(attempt.rules.stage(0) == Stage::Lifted, "early Q cannot accrue dwell before P completion");
    attempt.obs[0].twist = 0;
    attempt.update(4);
    require(attempt.rules.stage(0) == Stage::Flipped, "returning Q neutral permits P completion");
    attempt.obs[0].twist = 0.5;
    attempt.update(4.1);
    attempt.update(4.5);
    require(attempt.rules.confirm(0, 4.5), "P followed by Q and dwell succeeds");

    Attempt zero(3, {0, 0});
    zero.insert_both(1);
    zero.lift_both(2);
    zero.flip_both(3);
    zero.update(3.4);
    require(zero.rules.confirm(0, 3.4), "zero-degree Q target requires dwell but no artificial rotation");
}

void test_removal_death_and_card() {
    Attempt removal(3);
    removal.insert_both(1);
    removal.obs[0].insertion = 0;
    removal.update(2);
    require(removal.rules.failed(), "removal must immediately fail");

    Attempt death(1);
    death.obs[0].robot_alive = false;
    death.update(1);
    require(death.rules.failed(), "robot death must fail assembly");

    Attempt card(2);
    card.obs[0].card_present = false;
    card.update(1);
    card.update(15.9);
    require(!card.rules.failed(), "short card absence must not fail");
    card.obs[0].card_present = true;
    card.update(16);
    card.obs[0].card_present = false;
    card.update(17);
    card.update(31.999);
    require(!card.rules.failed(), "card reappearance must reset absence timer");
    card.update(32);
    require(card.rules.failed(), "continuous 15 second absence must fail");
}

void test_level4_success_and_step4_exemption() {
    Attempt attempt(4);
    attempt.obs[0].insertion = 0.1;
    attempt.update(1);
    attempt.obs[1].insertion = 0.1;
    attempt.update(6);
    attempt.obs[0].lift = 0.1;
    attempt.update(7);
    attempt.obs[1].lift = 0.1;
    attempt.update(12);
    attempt.obs[0].flip = pi / 2;
    attempt.update(13);
    attempt.obs[1].flip = pi / 2;
    attempt.update(20);
    require(!attempt.rules.failed(), "step 4 is explicitly exempt from five second pairing");
    attempt.align_both(21);
    require(attempt.rules.confirm(0, 22), "first core confirmation should be accepted");
    require(!attempt.rules.successful(), "both core confirmations are required");
    require(attempt.rules.confirm(1, 27), "second confirmation at five second boundary accepted");
    require(attempt.rules.successful(), "level 4 must succeed after both confirmations");
}

void test_every_level4_pair_deadline() {
    Attempt insertion(4);
    insertion.obs[0].insertion = 0.1;
    insertion.update(1);
    insertion.obs[1].insertion = 0.1;
    insertion.update(6.001);
    require(insertion.rules.failed(), "step 2 pairing deadline must be enforced");
    require(insertion.rules.failure_reason().find("step 2") != std::string::npos, "report step 2");
    require(std::abs(insertion.rules.retry_after() - 96.001) < 1e-8, "level 4 failure blocks 90 seconds");

    Attempt lift(4);
    lift.insert_both(1);
    lift.obs[0].lift = 0.1;
    lift.update(2);
    lift.update(7.001);
    require(lift.rules.failed(), "step 3 timeout must fail even without second completion");

    Attempt align(4);
    align.insert_both(1);
    align.lift_both(2);
    align.flip_both(3);
    align.obs[0].twist = 0.5;
    align.update(4);
    align.update(4.4);
    align.update(9.401);
    require(align.rules.failed(), "step 5 pairing deadline must be enforced");

    Attempt confirm(4);
    confirm.assemble();
    require(confirm.rules.confirm(0, 5), "first confirmation accepted");
    require(!confirm.rules.confirm(1, 10.001), "step 6 late confirmation rejected");
    require(confirm.rules.failed(), "step 6 pairing deadline must fail attempt");
}

void test_level4_arrival_deadline() {
    AssemblyRules rules(4, {0.5, -0.5});
    std::array<Observation, 2> obs{};
    obs[0].station_ready = false;
    obs[1].station_ready = false;
    rules.update(0, obs);
    rules.update(60, obs);
    require(!rules.failed(), "arrival deadline cannot start during station movement");
    obs[0].station_ready = true;
    rules.update(61, obs);
    obs[1].station_ready = true;
    rules.update(62, obs);
    require(rules.arrival_time() == 62, "both cores must arrive to start level 4 deadline");
    rules.update(107, obs);
    require(!rules.failed(), "45 second boundary remains allowed");
    rules.update(107.001, obs);
    require(rules.failed(), "assembly beyond 45 seconds must fail");
}

void test_confirmed_core_drift() {
    Attempt attempt(4);
    attempt.assemble();
    require(attempt.rules.confirm(0, 5), "first confirmation accepted");
    attempt.obs[0].lift = 0.08;
    attempt.update(5.1);
    require(attempt.rules.failed(), "confirmed first core cannot drift before paired success");
    require(!attempt.rules.confirm(1, 5.1), "second confirmation cannot bypass first core validity");
}

void test_difficulty_unlocks() {
    const std::array<bool, 4> none{};
    std::array<bool, 4> done = {true, true, true, false};
    require(AssemblyRules::difficulty_available(1, 0, none), "level 1 available at start");
    require(!AssemblyRules::difficulty_available(2, 60, none), "predecessor completion required");
    for (int level = 2; level <= 4; ++level) {
        const double unlock = 60 * (level - 1);
        require(!AssemblyRules::difficulty_available(level, unlock - .001, done), "time gate before unlock");
        require(AssemblyRules::difficulty_available(level, unlock, done), "exact unlock boundary");
    }
    done[3] = true;
    require(!AssemblyRules::difficulty_available(4, 180, done), "level 4 only one success per game");
}

void test_invalid_inputs() {
    require_throws([] { AssemblyRules rules(0); }, "invalid difficulty rejected");
    require_throws([] { AssemblyRules rules(3, {pi, 0}); }, "out of range Q target rejected");
    require_throws([] { AssemblyRules rules(3, {}, 0.2); }, "out of range display delay rejected");
    Attempt attempt(3);
    require_throws([&] { attempt.update(-1); }, "nonmonotonic time rejected");
    attempt.obs[0].twist = std::numeric_limits<double>::quiet_NaN();
    require_throws([&] { attempt.update(1); }, "nonfinite sensor input rejected");
    require_throws([&] { attempt.rules.confirm(1, 1); }, "inactive core rejected");
}
} // namespace

int main() {
    try {
        test_lower_difficulties_and_cooldown();
        test_early_insertion_and_sequence();
        test_dwell_and_lock();
        test_all_prerequisites_remain_live();
        test_repeated_confirmation_and_terminal_result();
        test_twist_follows_flip();
        test_removal_death_and_card();
        test_level4_success_and_step4_exemption();
        test_every_level4_pair_deadline();
        test_level4_arrival_deadline();
        test_confirmed_core_drift();
        test_difficulty_unlocks();
        test_invalid_inputs();
        std::cout << "13 rule test groups passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "rule test failed: " << error.what() << '\n';
        return 1;
    }
}
