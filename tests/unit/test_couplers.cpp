#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "scenario.h"
#include "vehicle.h"

#include "utilities/utilities.h"
#include "utilities/Globals.h"

// characterisation of the couplers between vehicles: coupling and uncoupling by hand (Attach, DettachStatus,
// Dettach), the automatic couplers coupling on contact, a broken coupler, and collisions which derail, on real
// vehicles, the moving ones recorded in golden traces (data/golden/couplers/)

using test::consist;
using test::definition;
using test::scenario;
using test::vehicle;
using namespace test::channels;

TEST_CASE("couplers: coupled by hand, touching or apart, of the same kind or not") {
	consist train;
	auto &locomotive{train.add(definition::fiz("4e_v1/4e")).mover()};
	// the wagon's buffers touch the locomotive's, the unit's car stands a metre away
	auto &wagon{train.add(definition::fiz("3wl_v2/3wl"), 0, coupling::faux).mover()};
	train.step();

	// nothing to couple to, or nothing to couple
	CHECK_FALSE(locomotive.Attach(end::rear, end::front, nullptr, coupling::coupler));
	CHECK_FALSE(locomotive.Attach(end::rear, end::front, &wagon, coupling::faux));
	// the hook first: the coupler alone, which sounds
	locomotive.Couplers[end::rear].sounds = 0;
	REQUIRE(locomotive.Attach(end::rear, end::front, &wagon, coupling::coupler));
	CHECK(locomotive.Couplers[end::rear].Connected == &wagon);
	CHECK(wagon.Couplers[end::front].Connected == &locomotive);
	CHECK(wagon.Couplers[end::front].CouplingFlag == coupling::coupler);
	CHECK(locomotive.Couplers[end::rear].sounds == sound::attachcoupler);
	// then the hoses: only what changes sounds
	locomotive.Couplers[end::rear].sounds = 0;
	REQUIRE(locomotive.Attach(end::rear, end::front, &wagon, coupling::coupler | coupling::brakehose | coupling::mainhose));
	CHECK(locomotive.Couplers[end::rear].sounds == (sound::attachbrakehose | sound::attachmainhose));
	// and again, quietly, the other side naming the coupler by the one it's connected to (2)
	locomotive.Couplers[end::rear].sounds = 0;
	REQUIRE(wagon.Attach(end::front, 2, &locomotive, coupling::coupler | coupling::brakehose, false, false));
	CHECK(locomotive.Couplers[end::rear].CouplingFlag == (coupling::coupler | coupling::brakehose));
	CHECK(locomotive.Couplers[end::rear].sounds == 0);

	// a screw coupler with the buffers pressed together comes apart whole
	train.step();
	CHECK(locomotive.DettachStatus(end::rear) < 0);
	locomotive.Couplers[end::rear].sounds = 0;
	CHECK(locomotive.Dettach(end::rear));
	CHECK(locomotive.Couplers[end::rear].Connected == nullptr);
	CHECK(wagon.Couplers[end::front].Connected == nullptr);
	CHECK(locomotive.Couplers[end::rear].sounds == (sound::detach | sound::attachcoupler | sound::attachbrakehose));
	CHECK(locomotive.DettachStatus(end::rear) == 0);
	CHECK(locomotive.Dettach(end::rear));
}

TEST_CASE("couplers: a screw coupler uncoupled with the buffers apart, or with the hoses alone, or broken") {
	struct state {
		char const *name;
		double gap; // between the buffers, the coupler stretched
		int coupling;
		bool damaged;
		bool apart; // whether it comes apart whole
	};
	for (auto const &[name, gap, couplingtype, damaged, apart] : {
	         state{"stretched a little", 0.1, coupling::coupler | coupling::brakehose, false, false},
	         state{"stretched far", 0.3, coupling::coupler | coupling::brakehose, false, true},
	         state{"hoses alone", 0.1, coupling::brakehose, false, true},
	         state{"broken", 0.1, coupling::coupler | coupling::brakehose, true, true},
	     }) {
		CAPTURE(name);
		consist train;
		auto &locomotive{train.add(definition::fiz("4e_v1/4e")).mover()};
		auto &wagon{train.add(definition::fiz("3wl_v2/3wl"), 0, coupling::faux, 0.0, gap).mover()};
		REQUIRE(locomotive.Attach(end::rear, end::front, &wagon, couplingtype, true));
		if (damaged) {
			SetFlag(locomotive.DamageFlag, dtrain_coupling);
		}
		train.step();
		CHECK((locomotive.DettachStatus(end::rear) < 0) == apart);
		CHECK(locomotive.Dettach(end::rear) == apart);
		// what's left when it doesn't come apart is the coupler, the hoses taken off
		CHECK(locomotive.Couplers[end::rear].CouplingFlag == (apart ? coupling::faux : coupling::coupler));
		CHECK(wagon.Couplers[end::front].CouplingFlag == (apart ? coupling::faux : coupling::coupler));
	}
	// couplers of different kinds don't couple, unless forced
	consist train;
	auto &locomotive{train.add(definition::fiz("4e_v1/4e")).mover()};
	auto &unit{train.add(definition::fiz("en57_v1/5bgb"), 0, coupling::faux).mover()};
	train.step();
	CHECK_FALSE(locomotive.Attach(end::rear, end::front, &unit, coupling::coupler));
	CHECK(locomotive.Attach(end::rear, end::front, &unit, coupling::coupler, true));
}

TEST_CASE("couplers: automatic couplers coupling on contact") {
	// a car running slowly into another from close by (its brakes are on), the automatic couplers of both ends
	// set to couple what their AutomaticFlag says; without one, the hook alone
	struct meeting {
		char const *name;
		char const *standing; // its rear coupler met
		char const *running; // its front coupler meets
		int coupling; // what they couple: what both flags allow, the control cable only of one kind
	};
	for (auto const &[name, standing, running, couplingtype] : {
	         meeting{"en57 to en57", "en57_v1/5bgb", "en57_v1/5bgaiii_led_spot", 39},
	         meeting{"impuls to en57", "en57_v1/5bgb", "impuls_v1/31we-a_pr", 39 & ~coupling::control},
	         meeting{"en57 without the flag", "en57_v1/5bgaiii_led_spot", "en57_v1/6ba_840", coupling::coupler},
	     }) {
		CAPTURE(name);
		scenario scenario{std::string{"couplers/automatic/"} + name, motion(), test::track{}};
		scenario.add(standing, 0, consist::full_coupling);
		scenario.add(running, 0, coupling::faux, 3.0, 0.2);
		scenario.wait(10.0);
		auto &first{scenario.vehicle(0)};
		auto &second{scenario.vehicle(1)};
		CHECK(first.Couplers[end::rear].CouplingFlag == couplingtype);
		CHECK(second.Couplers[end::front].CouplingFlag == couplingtype);
		// once coupled, not again until uncoupled; the vehicle which coupled marks it for the AI
		CHECK_FALSE(first.Couplers[end::rear].AutomaticCouplingAllowed);
		CHECK_FALSE(second.Couplers[end::front].AutomaticCouplingAllowed);
		CHECK((TestFlag(first.AIFlag, sound::attachcoupler) || TestFlag(second.AIFlag, sound::attachcoupler)));
		scenario.check();
	}
}

TEST_CASE("couplers: a coupler broken, with its brake hose torn off") {
	consist train;
	auto &locomotive{train.add(definition::fiz("4e_v1/4e")).mover()};
	auto &wagon{train.add(definition::fiz("3wl_v2/3wl")).mover()};
	REQUIRE(locomotive.Couplers[end::rear].Connected == &wagon);
	locomotive.damage_coupler(end::rear);
	CHECK(TestFlag(locomotive.DamageFlag, dtrain_coupling));
	CHECK(locomotive.AlarmChainFlag);
	CHECK(locomotive.Couplers[end::rear].CouplingFlag == coupling::faux);
	CHECK(wagon.Couplers[end::front].CouplingFlag == coupling::faux);
	// the front coupler of the wagon, broken on its own: no hose to tear
	wagon.damage_coupler(end::front);
	CHECK(TestFlag(wagon.DamageFlag, dtrain_coupling));
	CHECK_FALSE(wagon.AlarmChainFlag);
	// articulated couplers don't break
	consist articulated;
	auto &front{articulated.add(definition::fiz("impuls_v1/31we-a_pr")).mover()};
	articulated.add(definition::fiz("impuls_v1/31we-b_pr"));
	front.damage_coupler(end::rear);
	CHECK_FALSE(TestFlag(front.DamageFlag, dtrain_coupling));
	CHECK(front.Couplers[end::rear].CouplingFlag != coupling::faux);
}

TEST_CASE("couplers: collisions fast enough to derail") {
	// a locomotive running into a standing wagon from behind, and a wagon running backwards into a standing
	// locomotive; over 15 km/h, weighed by the ratio of the masses, the vehicles derail
	struct collision {
		char const *name;
		char const *front;
		double frontspeed;
		char const *back;
		double backspeed;
		bool frontout;
		bool backout;
	};
	for (auto const &[name, front, frontspeed, back, backspeed, frontout, backout] : {
	         collision{"locomotive into a wagon at 60", "3wl_v2/3wl", 0.0, "4e_v1/4e", 60.0, true, false},
	         collision{"wagon backwards into a locomotive at 60", "3wl_v2/3wl", -60.0, "4e_v1/4e", 0.0, true, false},
	         collision{"locomotive into a locomotive at 25", "4e_v1/4e", 0.0, "4e_v1/4e", 25.0, true, true},
	     }) {
		CAPTURE(name);
		scenario scenario{std::string{"couplers/collision/"} + name, motion(), test::track{}};
		scenario.add(front, 0, consist::full_coupling, frontspeed);
		scenario.add(back, 0, coupling::faux, backspeed, 20.0);
		scenario.wait(10.0);
		CHECK(TestFlag(scenario.vehicle(0).DamageFlag, dtrain_out) == frontout);
		CHECK(TestFlag(scenario.vehicle(1).DamageFlag, dtrain_out) == backout);
		scenario.check();
	}
}

namespace {

// the wagon from data/wagon.json with automatic couplers at both ends, which couple the given connections
definition automatic_definition(int Flag = coupling::coupler | coupling::brakehose) {
	return definition::load("wagon").set("BuffCoupl", "CType", "Automatic").set("BuffCoupl", "AutomaticFlag", Flag);
}

// the wagon from data/wagon.json with automatic couplers at both ends, running at the given speed [km/h]
vehicle automatic_wagon(double Velocity, std::string const &Name) {
	return vehicle{automatic_definition(), Velocity, Name};
}

// the same wagon, twice as heavy
vehicle heavy_automatic_wagon(double Velocity, std::string const &Name) {
	return vehicle{automatic_definition().set("Param", "M", 2 * 42200), Velocity, Name};
}

bool derailed(vehicle &Vehicle) {
	return TestFlag(Vehicle->DamageFlag, dtrain_out);
}

// runs the pair for Seconds, returns whether the vehicles were coupled at any point
bool couple_during(test::pair &Pair, double Seconds) {
	auto coupled{false};
	for (auto i{std::lround(Seconds / 0.01)}; i > 0; --i) {
		Pair.step();
		coupled = coupled || Pair.coupled();
	}
	return coupled;
}

// sets the crash damage setting for the test, restores it after
class crash_damage_setting {
public:
	explicit crash_damage_setting(bool Enabled) { Global.crash_damage = Enabled; }
	~crash_damage_setting() { Global.crash_damage = m_previous; }
	crash_damage_setting(crash_damage_setting const &) = delete;
	crash_damage_setting &operator=(crash_damage_setting const &) = delete;

private:
	bool m_previous{Global.crash_damage};
};

} // namespace

// issue #86: automatic couplers used to catch at any closing speed, so the vehicles never collided nor derailed

TEST_CASE("couplers: automatic couplers catch a standing wagon hit at a walking pace") {
	crash_damage_setting const crashdamage{true};
	auto moving{automatic_wagon(10.0, "moving")};
	auto standing{automatic_wagon(0.0, "standing")};
	REQUIRE(moving->Couplers[end::front].type() == TCouplerType::Automatic);
	REQUIRE(moving.speed() == doctest::Approx(10.0));

	// the front of the moving wagon runs into the rear of the standing one
	CHECK(moving->CollisionVelocity(end::front, standing.mover(), end::rear) == doctest::Approx(10.0));
	CHECK(moving->AutomaticCouplingSafe(end::front, standing.mover(), end::rear));
	CHECK(standing->AutomaticCouplingSafe(end::rear, moving.mover(), end::front));
}

TEST_CASE("couplers: automatic couplers don't catch a wagon hit too fast, the wagons collide instead") {
	crash_damage_setting const crashdamage{true};
	auto moving{automatic_wagon(30.0, "moving")};
	auto standing{automatic_wagon(0.0, "standing")};
	REQUIRE(moving.speed() == doctest::Approx(30.0));

	CHECK(moving->CollisionVelocity(end::front, standing.mover(), end::rear) > TMoverParameters::SafeCollisionVelocity);
	CHECK_FALSE(moving->AutomaticCouplingSafe(end::front, standing.mover(), end::rear));
	CHECK_FALSE(standing->AutomaticCouplingSafe(end::rear, moving.mover(), end::front));
}

TEST_CASE("couplers: automatic couplers catch a wagon running ahead only a little slower") {
	crash_damage_setting const crashdamage{true};
	auto faster{automatic_wagon(60.0, "faster")};
	auto slower{automatic_wagon(50.0, "slower")};

	CHECK(faster->CollisionVelocity(end::front, slower.mover(), end::rear) == doctest::Approx(10.0));
	CHECK(slower->CollisionVelocity(end::rear, faster.mover(), end::front) == doctest::Approx(10.0));
	CHECK(faster->AutomaticCouplingSafe(end::front, slower.mover(), end::rear));
}

TEST_CASE("couplers: without crash damage automatic couplers catch at any speed") {
	crash_damage_setting const crashdamage{false};
	auto moving{automatic_wagon(30.0, "moving")};
	auto standing{automatic_wagon(0.0, "standing")};

	CHECK(moving->AutomaticCouplingSafe(end::front, standing.mover(), end::rear));
}

// the closing speed used to treat the speed of each vehicle as a vector turned by half its heading, so for vehicles
// facing each other it came out as the root of the sum of the squares of their speeds, rather than the sum

TEST_CASE("couplers: the closing speed of a rear-end collision is the difference of the speeds") {
	auto faster{automatic_wagon(40.0, "faster")};
	auto slower{automatic_wagon(25.0, "slower")};

	// running forwards, the front of the faster wagon hits the rear of the slower one
	CHECK(faster->CollisionVelocity(end::front, slower.mover(), end::rear) == doctest::Approx(15.0));
	CHECK(slower->CollisionVelocity(end::rear, faster.mover(), end::front) == doctest::Approx(15.0));
	// the wagons running ahead of each other are parting
	CHECK(slower->CollisionVelocity(end::front, faster.mover(), end::rear) == doctest::Approx(-15.0));

	// backing up, the rear of the faster wagon hits the front of the slower one
	auto backingfaster{automatic_wagon(-40.0, "backing faster")};
	auto backingslower{automatic_wagon(-25.0, "backing slower")};
	CHECK(backingfaster->CollisionVelocity(end::rear, backingslower.mover(), end::front) == doctest::Approx(15.0));
	CHECK(backingslower->CollisionVelocity(end::front, backingfaster.mover(), end::rear) == doctest::Approx(15.0));
}

TEST_CASE("couplers: the closing speed of a head-on collision is the sum of the speeds") {
	crash_damage_setting const crashdamage{true};
	auto first{automatic_wagon(10.0, "first")};
	auto second{automatic_wagon(10.0, "second")};

	// both wagons run forwards, their fronts meet
	CHECK(first->CollisionVelocity(end::front, second.mover(), end::front) == doctest::Approx(20.0));
	CHECK(second->CollisionVelocity(end::front, first.mover(), end::front) == doctest::Approx(20.0));
	// 20 km/h is above the limit, the couplers don't catch (the old formula gave 14.1 km/h and coupled them)
	CHECK(first->CollisionVelocity(end::front, second.mover(), end::front) > TMoverParameters::SafeCollisionVelocity);
	CHECK_FALSE(first->AutomaticCouplingSafe(end::front, second.mover(), end::front));
	CHECK_FALSE(second->AutomaticCouplingSafe(end::front, first.mover(), end::front));

	// both wagons back up, their rears meet
	auto firstbacking{automatic_wagon(-10.0, "first backing")};
	auto secondbacking{automatic_wagon(-10.0, "second backing")};
	CHECK(firstbacking->CollisionVelocity(end::rear, secondbacking.mover(), end::rear) == doctest::Approx(20.0));

	// running back to back, the wagons part
	CHECK(first->CollisionVelocity(end::rear, second.mover(), end::rear) == doctest::Approx(-20.0));
	CHECK(first->AutomaticCouplingSafe(end::rear, second.mover(), end::rear));

	// slow enough, the couplers catch
	auto slowfirst{automatic_wagon(5.0, "slow first")};
	auto slowsecond{automatic_wagon(5.0, "slow second")};
	CHECK(slowfirst->CollisionVelocity(end::front, slowsecond.mover(), end::front) == doctest::Approx(10.0));
	CHECK(slowfirst->AutomaticCouplingSafe(end::front, slowsecond.mover(), end::front));
}

TEST_CASE("couplers: the closing speed of a hit on a standing vehicle is the speed of the moving one, whichever way it faces") {
	auto moving{automatic_wagon(20.0, "moving")};
	auto standing{automatic_wagon(0.0, "standing")};

	// the standing wagon faces away from the moving one
	CHECK(moving->CollisionVelocity(end::front, standing.mover(), end::rear) == doctest::Approx(20.0));
	CHECK(standing->CollisionVelocity(end::rear, moving.mover(), end::front) == doctest::Approx(20.0));
	// the standing wagon faces the moving one
	CHECK(moving->CollisionVelocity(end::front, standing.mover(), end::front) == doctest::Approx(20.0));
	CHECK(standing->CollisionVelocity(end::front, moving.mover(), end::front) == doctest::Approx(20.0));
	// the moving wagon backs up into the standing one
	auto backing{automatic_wagon(-20.0, "backing")};
	CHECK(backing->CollisionVelocity(end::rear, standing.mover(), end::front) == doctest::Approx(20.0));
	CHECK(backing->CollisionVelocity(end::rear, standing.mover(), end::rear) == doctest::Approx(20.0));
	// the moving wagon runs away from the standing one
	CHECK(moving->CollisionVelocity(end::rear, standing.mover(), end::front) == doctest::Approx(-20.0));
}

// the coupling as the simulation runs it: two vehicles on a track, closing in on each other until the couplers (or
// buffers) meet, the coupler forces and the collision computed by the physics, step by step

TEST_CASE("couplers: automatic couplers catch right up to the safe closing speed") {
	crash_damage_setting const crashdamage{true};
	auto standing{automatic_wagon(0.0, "standing")};

	auto below{automatic_wagon(14.9, "below")};
	CHECK(below->CollisionVelocity(end::front, standing.mover(), end::rear) == doctest::Approx(14.9));
	CHECK(below->AutomaticCouplingSafe(end::front, standing.mover(), end::rear));

	auto above{automatic_wagon(15.1, "above")};
	CHECK(above->CollisionVelocity(end::front, standing.mover(), end::rear) == doctest::Approx(15.1));
	CHECK_FALSE(above->AutomaticCouplingSafe(end::front, standing.mover(), end::rear));
	auto faster{automatic_wagon(15.5, "faster")};
	CHECK_FALSE(faster->AutomaticCouplingSafe(end::front, standing.mover(), end::rear));
}

TEST_CASE("couplers: a wagon rolling slowly into a standing one couples with it") {
	crash_damage_setting const crashdamage{true};
	auto moving{automatic_wagon(5.0, "moving")};
	auto standing{automatic_wagon(0.0, "standing")};
	test::pair pair{moving, end::front, standing, end::rear, 1.0};

	pair.run(3.0);

	REQUIRE(pair.coupled());
	CHECK(moving->Couplers[end::front].CouplingFlag == (coupling::coupler | coupling::brakehose));
	CHECK(standing->Couplers[end::rear].CouplingFlag == (coupling::coupler | coupling::brakehose));
	CHECK(moving->Couplers[end::front].Connected == &standing.mover());
	CHECK_FALSE(derailed(moving));
	CHECK_FALSE(derailed(standing));
	// coupled, they run on together
	pair.run(2.0);
	CHECK(moving.speed() == doctest::Approx(standing.speed()).epsilon(0.05));
	CHECK(pair.gap() < 0.2);
}

TEST_CASE("couplers: a wagon hitting a standing one too fast doesn't couple, nor once the hit has slowed it down") {
	crash_damage_setting const crashdamage{true};
	auto moving{automatic_wagon(30.0, "moving")};
	auto standing{automatic_wagon(0.0, "standing")};
	test::pair pair{moving, end::front, standing, end::rear, 1.0};

	auto touched{false};
	auto parted{false};
	auto coupled{false};
	auto locked{true};
	for (int i = 0; i < 1000; ++i) {
		pair.step();
		// as far as the couplers know: the distance between them at the start of the step
		auto const distance{moving->Neighbours[end::front].distance};
		touched = touched || distance < 0.0;
		parted = parted || (touched && distance > 0.05);
		coupled = coupled || pair.coupled();
		if (touched && false == parted) {
			locked = locked && false == moving->Couplers[end::front].AutomaticCouplingAllowed && false == standing->Couplers[end::rear].AutomaticCouplingAllowed;
		}
	}

	REQUIRE(touched);
	CHECK_FALSE(coupled);
	// the hit derailed and stopped them, the couplers next to each other, the automatic coupling locked all along
	CHECK_FALSE(parted);
	CHECK(locked);
	CHECK(derailed(moving));
	CHECK(derailed(standing));
	CHECK(std::abs(moving.speed()) < 3.0);
	CHECK(std::abs(standing.speed()) < 3.0);
	CHECK_FALSE(couple_during(pair, 5.0));
}

TEST_CASE("couplers: the automatic coupling locked by a hit is unlocked once the vehicles part") {
	crash_damage_setting const crashdamage{true};
	auto moving{automatic_wagon(3.0, "moving")};
	auto standing{automatic_wagon(0.0, "standing")};
	// as a hit too fast or manual uncoupling leaves them
	moving->Couplers[end::front].AutomaticCouplingAllowed = false;
	standing->Couplers[end::rear].AutomaticCouplingAllowed = false;

	SUBCASE("parted by more than 5 cm, they couple on the next approach") {
		test::pair pair{moving, end::front, standing, end::rear, 0.5};
		pair.step();
		CHECK(moving->Couplers[end::front].AutomaticCouplingAllowed);
		CHECK(couple_during(pair, 3.0));
	}
	SUBCASE("kept together, they don't") {
		test::pair pair{moving, end::front, standing, end::rear, 0.0};
		auto coupled{false};
		for (int i = 0; i < 300; ++i) {
			pair.step();
			if (pair.gap() > 0.05) {
				break; // parted
			}
			coupled = coupled || pair.coupled();
		}
		CHECK_FALSE(coupled);
	}
}

// the couplers refuse a hit by the closing speed when they meet, the derail check (CollisionDetect()) takes the one
// left when the buffers are pressed in all the way, after their springs have slowed the hit down: for the wagons here
// by about 7 km/h. so a hit a little over the safe closing speed only bounces the vehicles apart

TEST_CASE("couplers: a hit a little too fast for the couplers bounces the vehicles apart, uncoupled and on the rails") {
	crash_damage_setting const crashdamage{true};
	auto moving{automatic_wagon(20.0, "moving")};
	auto standing{automatic_wagon(0.0, "standing")};
	test::pair pair{moving, end::front, standing, end::rear, 1.0};

	CHECK_FALSE(couple_during(pair, 3.0));
	CHECK_FALSE(derailed(moving));
	CHECK_FALSE(derailed(standing));
	CHECK(pair.gap() > 0.5);
	CHECK(standing.speed() > moving.speed());
}

TEST_CASE("couplers: a hit too fast for the couplers derails the vehicles over the limit adjusted for their mass ratio") {
	crash_damage_setting const crashdamage{true};

	SUBCASE("equal masses: both derail") {
		auto moving{automatic_wagon(30.0, "moving")};
		auto standing{automatic_wagon(0.0, "standing")};
		test::pair pair{moving, end::front, standing, end::rear, 1.0};
		CHECK_FALSE(couple_during(pair, 5.0));
		CHECK(derailed(moving));
		CHECK(derailed(standing));
	}
	SUBCASE("half the mass: between the safe closing speed and twice that, only the lighter one derails") {
		auto light{automatic_wagon(27.0, "light")};
		auto heavy{heavy_automatic_wagon(0.0, "heavy")};
		test::pair pair{light, end::front, heavy, end::rear, 1.0};
		CHECK_FALSE(couple_during(pair, 5.0));
		CHECK(derailed(light));
		CHECK_FALSE(derailed(heavy));
	}
	SUBCASE("half the mass: above twice the safe closing speed both derail") {
		auto light{automatic_wagon(45.0, "light")};
		auto heavy{heavy_automatic_wagon(0.0, "heavy")};
		test::pair pair{light, end::front, heavy, end::rear, 1.0};
		CHECK_FALSE(couple_during(pair, 5.0));
		CHECK(derailed(light));
		CHECK(derailed(heavy));
	}
	SUBCASE("half the mass: below the safe closing speed the couplers catch, nothing derails") {
		auto light{automatic_wagon(12.0, "light")};
		auto heavy{heavy_automatic_wagon(0.0, "heavy")};
		test::pair pair{light, end::front, heavy, end::rear, 1.0};
		CHECK(couple_during(pair, 5.0));
		CHECK_FALSE(derailed(light));
		CHECK_FALSE(derailed(heavy));
	}
}

TEST_CASE("couplers: wagons running head-on into each other couple only up to the safe closing speed") {
	crash_damage_setting const crashdamage{true};

	SUBCASE("7 + 7 km/h, the couplers catch") {
		auto first{automatic_wagon(7.0, "first")};
		auto second{automatic_wagon(7.0, "second")};
		test::pair pair{first, end::front, second, end::front, 1.0};
		CHECK(couple_during(pair, 3.0));
		CHECK_FALSE(derailed(first));
		CHECK_FALSE(derailed(second));
	}
	SUBCASE("8 + 8 km/h, the wagons bounce apart") {
		auto first{automatic_wagon(8.0, "first")};
		auto second{automatic_wagon(8.0, "second")};
		test::pair pair{first, end::front, second, end::front, 1.0};
		CHECK_FALSE(couple_during(pair, 3.0));
		CHECK_FALSE(derailed(first));
		CHECK_FALSE(derailed(second));
		// both backwards, away from each other
		CHECK(first.speed() < 0.0);
		CHECK(second.speed() < 0.0);
	}
	SUBCASE("backing up, 8 + 8 km/h, the wagons bounce apart") {
		auto first{automatic_wagon(-8.0, "first")};
		auto second{automatic_wagon(-8.0, "second")};
		test::pair pair{first, end::rear, second, end::rear, 1.0};
		CHECK_FALSE(couple_during(pair, 3.0));
		CHECK(first.speed() > 0.0);
		CHECK(second.speed() > 0.0);
	}
	SUBCASE("15 + 15 km/h, both derail") {
		auto first{automatic_wagon(15.0, "first")};
		auto second{automatic_wagon(15.0, "second")};
		test::pair pair{first, end::front, second, end::front, 1.0};
		CHECK_FALSE(couple_during(pair, 5.0));
		CHECK(derailed(first));
		CHECK(derailed(second));
	}
}

// the collision used to take the speed of a vehicle facing the other way as if it faced the same way, so a wagon
// pushed away by the buffers of the vehicle hitting it head-on counted as running into it: the hit slowed it down,
// and the hitter bounced back as from a wall

TEST_CASE("couplers: a wagon hit head-on runs off the same as one hit from behind") {
	// without automatic couplers or crash damage, the collision alone
	crash_damage_setting const crashdamage{false};
	auto speeds_after_hit = [](int StruckEnd) {
		auto moving{vehicle{definition::load("wagon"), 20.0, "moving"}};
		auto struck{vehicle{definition::load("wagon"), 0.0, "struck"}};
		test::pair pair{moving, end::front, struck, StruckEnd, 0.5};
		auto touched{false};
		for (int i = 0; i < 300; ++i) {
			pair.step();
			touched = touched || pair.gap() < 0.0;
			if (touched && pair.gap() > 0.3) {
				break; // bounced off
			}
		}
		REQUIRE(touched);
		REQUIRE(pair.gap() > 0.3);
		REQUIRE_FALSE(pair.coupled());
		return std::make_pair(moving.speed(), struck.speed());
	};

	auto const [hitterfrombehind, struckfrombehind]{speeds_after_hit(end::rear)};
	auto const [hitterheadon, struckheadon]{speeds_after_hit(end::front)};

	// hit from behind, the struck wagon runs off forwards, faster than the hitter
	CHECK(struckfrombehind > 10.0);
	CHECK(hitterfrombehind < struckfrombehind);
	// hit head-on, it runs off backwards, as fast
	CHECK(struckheadon == doctest::Approx(-struckfrombehind).epsilon(0.02));
	CHECK(hitterheadon == doctest::Approx(hitterfrombehind).epsilon(0.05));
}

// the other conditions of automatic coupling

TEST_CASE("couplers: an automatic coupler doesn't catch a coupler of another type") {
	crash_damage_setting const crashdamage{true};
	auto moving{automatic_wagon(5.0, "moving")};
	auto standing{vehicle{definition::load("wagon"), 0.0, "standing"}};
	REQUIRE(standing->Couplers[end::rear].type() == TCouplerType::Screw);
	test::pair pair{moving, end::front, standing, end::rear, 1.0};

	CHECK_FALSE(couple_during(pair, 3.0));
}

TEST_CASE("couplers: automatic couplers of different control systems couple without the control connection") {
	crash_damage_setting const crashdamage{true};
	auto const flag{coupling::coupler | coupling::brakehose | coupling::control};
	auto moving{vehicle{automatic_definition(flag).set("BuffCoupl", "ControlType", "first"), 5.0, "moving"}};

	SUBCASE("the same control system") {
		auto standing{vehicle{automatic_definition(flag).set("BuffCoupl", "ControlType", "first"), 0.0, "standing"}};
		REQUIRE(standing->Couplers[end::rear].control_type == "first");
		test::pair pair{moving, end::front, standing, end::rear, 1.0};
		REQUIRE(couple_during(pair, 3.0));
		CHECK(pair.coupling() == flag);
	}
	SUBCASE("a different one") {
		auto standing{vehicle{automatic_definition(flag).set("BuffCoupl", "ControlType", "second"), 0.0, "standing"}};
		REQUIRE(standing->Couplers[end::rear].control_type == "second");
		test::pair pair{moving, end::front, standing, end::rear, 1.0};
		REQUIRE(couple_during(pair, 3.0));
		CHECK(pair.coupling() == (coupling::coupler | coupling::brakehose));
	}
}

TEST_CASE("couplers: the coupler force of vehicles facing each other lands on the couplers holding them") {
	// the force a vehicle takes from its coupler, the other one takes back on its own coupler of the pair, whichever
	// way the vehicles face; it used to land on the far end of the other vehicle when they faced each other.
	// CForce runs along the axis of its own vehicle, so the two forces of the pair are opposite along the track: of
	// opposite signs for vehicles facing the same way, of the same sign for vehicles facing each other
	auto first{automatic_wagon(5.0, "first")};
	auto second{automatic_wagon(0.0, "second")};

	SUBCASE("facing each other") {
		test::pair pair{first, end::front, second, end::front, 1.0};
		pair.run(5.0);
		REQUIRE(pair.coupled());

		pair.step();
		REQUIRE(second->Couplers[end::front].CForce != 0.0);
		CHECK(first->Couplers[end::front].CForce == doctest::Approx(second->Couplers[end::front].CForce));
		CHECK(first->Couplers[end::rear].CForce == 0.0); // nothing there
		CHECK(second->Couplers[end::rear].CForce == 0.0);
	}
	SUBCASE("facing the same way") {
		test::pair pair{first, end::front, second, end::rear, 1.0};
		pair.run(5.0);
		REQUIRE(pair.coupled());

		pair.step();
		REQUIRE(second->Couplers[end::rear].CForce != 0.0);
		CHECK(first->Couplers[end::front].CForce == doctest::Approx(-second->Couplers[end::rear].CForce));
		CHECK(first->Couplers[end::rear].CForce == 0.0);
		CHECK(second->Couplers[end::front].CForce == 0.0);
	}
}
