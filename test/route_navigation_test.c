#include "route_runner.h"
#include "navigation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t rng = 1;
static float noise(void)
{
    /* Deterministic on every host: xorshift32 + Box-Muller. */
    float u[2];
    for (int i = 0; i < 2; i++) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        u[i] = ((rng >> 8) + 1.0f) / 16777217.0f;
    }
    return sqrtf(-2 * logf(u[0])) * cosf(6.2831853f * u[1]);
}

/* Dashboard-style polygon (edges cut into <= 700 mm pieces) driven with the
   settings read from the robot on 15.10.2026: auto power below the motor
   minimum, 6 Hz poses, 40 mm / 2 deg pose noise and 15 % weak turning. The
   robot starts beside the start point, so the map-path approach is included. */
static bool noisy_patrol(uint32_t seed)
{
    motion_settings_t cfg = {.auto_stop_mm=150,.resume_margin_mm=100,.auto_slow_mm=350,.auto_rotate_mm=65,
        .auto_drive_percent=80,.auto_slow_percent=80,.auto_turn_percent=80,.steer_min_percent=80,
        .drive_min_percent=85,.turn_min_percent=80,.drive_kick_ms=300,.turn_kick_ms=150,.turn_ramp_ms=400,
        .ramp_ms=1000,.turn_ms=1600,.reverse_ms=400};
    route_point_t corners[] = {{-800,-800},{800,-800},{800,800},{-800,800}};
    route_definition_t d = {.generation=1,.laps=2,.adaptive=true};
    for (unsigned i=0;i<4;i++) {
        route_point_t a=corners[i],b=corners[(i+1)%4];
        for (unsigned j=0;j<3;j++)
            d.points[d.count++]=(route_point_t){(int16_t)(a.x+(b.x-a.x)*(int)j/3),(int16_t)(a.y+(b.y-a.y)*(int)j/3)};
    }
    assert(!route_definition_error(&d));
    rng = seed;
    route_runner_t route={0};route_runner_start(&route,&d,0);
    navigation_auto_t navigation;
    navigation_auto_configure(&navigation,NAV_STRATEGY_PATH,d.points[0].x,d.points[0].y);
    navigation_view_t view={.front_known=true,.rear_known=true,.rotation_known=true,.front_mm=4000,.rear_mm=4000,
        .rear_free_mm=3500,.swing_mm=2000,.pose_valid=true,.route_valid=true,.heading_live=true};
    for(unsigned i=0;i<NAV_HEADINGS;i++)view.free_mm[i]=3500;
    motion_axis_t drive={0},turn={0};
    float x=-1300,y=-800,theta=0.5f,px=x,py=y;
    int actual_turn=0;
    for(uint64_t ms=0;ms<400000;ms+=20) {
        if(ms%160==0){px=x+40*noise();py=y+40*noise();view.pose_sequence++;}
        view.pose_x=px;view.pose_y=py;view.pose_theta=theta+0.035f*noise();view.yaw_rate=-actual_turn*.0148f;
        const char *reason;
        route_result_t result=route_runner_step(&route,(route_sample_t){.x=px,.y=py,.quality=1,
            .sequence=view.pose_sequence,.generation=1,.now_ms=ms,.healthy=true},&reason);
        if(result==ROUTE_DONE)return route.completed==2;
        if(result==ROUTE_FAILED){printf("Noisy patrol seed %lu failed: %s\n",(unsigned long)seed,reason);return false;}
        int drive_command=0,turn_command=0;bool precision=false,tight=false;
        if(result!=ROUTE_WAIT) {
            route_point_t goal=d.points[route.target];
            if(result==ROUTE_NEXT)navigation_auto_configure(&navigation,NAV_STRATEGY_ROUTE,goal.x,goal.y);
            view.route_goal_x=view.route_way_x=goal.x;view.route_goal_y=view.route_way_y=goal.y;
            motion_settings_t local=cfg;
            local.auto_drive_percent=route_runner_power(&route,cfg,1,3500,false);
            navigation_plan_t plan=navigation_auto_step(&navigation,&view,local,ms,view.pose_sequence);
            if(plan.done) {
                /* Same hand-off as robot_manager: the approach path reached its goal. */
                const char *arrival;
                if(!route.approaching||strcmp(plan.reason,"goal_reached")||
                   route_runner_approach_done(&route,&arrival)!=ROUTE_NEXT)return false;
                route_point_t next=d.points[route.target];
                navigation_auto_configure(&navigation,NAV_STRATEGY_ROUTE,next.x,next.y);
            } else {drive_command=plan.drive;turn_command=plan.turn;precision=plan.precision;tight=plan.tight;}
        }
        int actual_drive=motion_axis_step(&drive,drive_command,cfg.drive_min_percent,tight?250:cfg.drive_kick_ms,cfg.ramp_ms,20);
        actual_turn=motion_axis_step(&turn,turn_command,cfg.turn_min_percent,precision?20:tight?250:cfg.turn_kick_ms,
                                     precision?0:cfg.turn_ramp_ms,20);
        theta-=actual_turn*.0148f*.02f*0.85f;
        x+=actual_drive*2.0f*.02f*cosf(theta);y+=actual_drive*2.0f*.02f*sinf(theta);
    }
    return false;
}

/* Closed-loop route + real navigation + output-stage simulation. This is an
   ideal open-room model, not a measured hardware accuracy claim. */
int main(void)
{
    motion_settings_t cfg = {.auto_stop_mm=100,.resume_margin_mm=50,.auto_slow_mm=300,
        .auto_drive_percent=95,.auto_slow_percent=85,.auto_turn_percent=100,
        .steer_min_percent=80,.drive_min_percent=85,.turn_min_percent=80,
        .drive_kick_ms=1000,.turn_kick_ms=300,.turn_ramp_ms=400,.ramp_ms=1000,
        .turn_ms=1600,.reverse_ms=500,.auto_rotate_mm=40};
    route_definition_t definition = {.generation=1,.laps=4,.count=32,.adaptive=true};
    for (unsigned i=0;i<definition.count;i++) {
        float angle=6.28318530718f*i/definition.count;
        definition.points[i]=(route_point_t){(int16_t)lroundf(1600*cosf(angle)),(int16_t)lroundf(1600*sinf(angle))};
    }
    assert(!route_definition_error(&definition));
    route_runner_t route={0};route_runner_start(&route,&definition,0);
    navigation_auto_t navigation;
    navigation_auto_configure(&navigation,NAV_STRATEGY_PATH,definition.points[0].x,definition.points[0].y);
    navigation_view_t view={.front_known=true,.rear_known=true,.rotation_known=true,
        .front_mm=4000,.rear_mm=4000,.rear_free_mm=3500,.swing_mm=2000,.pose_valid=true,.route_valid=true,.heading_live=true};
    for(unsigned i=0;i<NAV_HEADINGS;i++)view.free_mm[i]=3500;
    motion_axis_t drive={0},turn={0};
    float x=1600,y=0,theta=1.57079632679f,max_radial=0;
    int actual_turn=0,highest=0;bool finished=false;
    for(uint64_t ms=0;ms<500000;ms+=20) {
        view.pose_sequence=(uint32_t)(ms/100+1);view.pose_x=x;view.pose_y=y;
        view.pose_theta=theta;view.yaw_rate=-actual_turn*.0148f;
        const char *reason;
        route_result_t result=route_runner_step(&route,(route_sample_t){.x=x,.y=y,.quality=1,
            .sequence=view.pose_sequence,.generation=1,.now_ms=ms,.healthy=true},&reason);
        if(result==ROUTE_DONE){finished=true;break;}
        if(result==ROUTE_FAILED){printf("Route failed: %s at %.0f %.0f, lap %lu target %u\n",reason,x,y,(unsigned long)route.completed,route.target);assert(false);}
        int drive_command=0,turn_command=0;bool precision=false,tight=false;
        if(result!=ROUTE_WAIT) {
            route_point_t goal=definition.points[route.target];
            if(result==ROUTE_NEXT)navigation_auto_configure(&navigation,NAV_STRATEGY_ROUTE,goal.x,goal.y);
            view.route_goal_x=view.route_way_x=goal.x;view.route_goal_y=view.route_way_y=goal.y;
            motion_settings_t local=cfg;
            local.auto_drive_percent=route_runner_power(&route,cfg,1,3500,false);
            if(local.auto_drive_percent>highest)highest=local.auto_drive_percent;
            navigation_plan_t plan=navigation_auto_step(&navigation,&view,local,ms,view.pose_sequence);
            assert(!plan.done);
            drive_command=plan.drive;turn_command=plan.turn;precision=plan.precision;tight=plan.tight;
            if(!strncmp(plan.reason,"escape",6)||!strncmp(plan.reason,"wiggle",6)||!strcmp(plan.reason,"path_avoid"))route_runner_trouble(&route);
        }
        int drive_kick=tight?250:cfg.drive_kick_ms;
        int turn_kick=precision?20:tight?250:cfg.turn_kick_ms;
        int actual_drive=motion_axis_step(&drive,drive_command,cfg.drive_min_percent,drive_kick,cfg.ramp_ms,20);
        actual_turn=motion_axis_step(&turn,turn_command,cfg.turn_min_percent,turn_kick,precision?0:cfg.turn_ramp_ms,20);
        theta-=actual_turn*.0148f*.02f;
        x+=actual_drive*2.0f*.02f*cosf(theta);y+=actual_drive*2.0f*.02f*sinf(theta);
        float radial=fabsf(hypotf(x,y)-1600);if(radial>max_radial)max_radial=radial;
    }
    printf("Closed-loop patrol: laps %lu, learned %lu, confidence %u/100, highest cruise %d%%, radial error %.0f mm (synthetic).\n",
        (unsigned long)route.completed,(unsigned long)route.learned_laps,route_runner_confidence(&route),highest,max_radial);
    assert(finished&&route.completed==4&&!route.active&&route.learned_laps>=2&&highest>85&&highest<=95&&max_radial<200);
    /* Direct patrol waypoints must not use the old 250-mm point-goal shortcut. */
    navigation_auto_configure(&navigation,NAV_STRATEGY_ROUTE,300,0);
    view.pose_x=100;view.pose_y=0;view.pose_theta=0;view.route_goal_x=view.route_way_x=300;view.route_goal_y=view.route_way_y=0;
    assert(!navigation_auto_step(&navigation,&view,cfg,0,1).done);
    view.map_weak=true;
    navigation_plan_t stopped=navigation_auto_step(&navigation,&view,cfg,20,2);
    assert(!stopped.drive&&!stopped.turn&&!stopped.done);
    for(uint32_t seed=1;seed<=8;seed++)assert(noisy_patrol(seed*2654435761u));
    puts("Patrol navigation: feedback, output stage, lap completion, localization hold and noisy square with robot settings passed.");
    return 0;
}
