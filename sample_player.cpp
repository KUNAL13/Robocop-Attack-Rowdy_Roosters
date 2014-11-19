

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

//bin/rione_player -team Ri-one -f conf/formations.conf -c conf/player.conf -num 1


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "sample_player.h"

#include "strategy.h"
#include "field_analyzer.h"

#include "action_chain_holder.h"
#include "sample_field_evaluator.h"

#include "soccer_role.h"

#include "sample_communication.h"
#include "keepaway_communication.h"

#include "bhv_penalty_kick.h"
#include "bhv_set_play.h"
#include "bhv_set_play_kick_in.h"
#include "bhv_set_play_indirect_free_kick.h"
#include "bhv_basic_move.h"

#include "bhv_custom_before_kick_off.h"
#include "bhv_strict_check_shoot.h"

#include "view_tactical.h"

#include "intention_receive.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/bhv_emergency.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/body_intercept.h>
#include <rcsc/action/body_smart_kick.h>
#include <rcsc/action/body_kick_one_step.h>
#include <rcsc/action/body_kick_to_relative.h>

#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/neck_turn_to_point.h>
#include <rcsc/action/view_synch.h>
#include <rcsc/action/body_hold_ball.h>
#include <rcsc/action/body_dribble.h>

#include <rcsc/formation/formation.h>
#include <rcsc/action/kick_table.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/say_message_builder.h>
#include <rcsc/player/audio_sensor.h>
#include <rcsc/player/freeform_parser.h>
#include <rcsc/player/debug_client.h>

#include "bhv_chain_action.h"
#include <rcsc/action/body_advance_ball.h>
#include <rcsc/action/body_dribble.h>
#include <rcsc/action/body_hold_ball.h>
#include <rcsc/action/body_pass.h>
#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>
#include <rcsc/common/basic_client.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/common/player_param.h>
#include <rcsc/common/audio_memory.h>
#include <rcsc/common/say_message_parser.h>
// #include <rcsc/common/free_message_parser.h>
#include <rcsc/geom/line_2d.h>

#include <rcsc/param/param_map.h>
#include <rcsc/param/cmd_line_parser.h>

#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <queue>
#define DEBUG_PRINT
using namespace rcsc;
using namespace std;
/*-------------------------------------------------------------------*/
/*!

 */
#define DEBUGDRAW
const string ATTACK_FORMATION_CONF = "offense-formation.conf";
//followDribbler
Formation::Ptr o_formation;
vector<Vector2D > p_positions;
double slope(double x1,double y1,double x2,double y2);
double AngleBetweenPoints(double x1,double y1,double x2,double y2,double x3,double y3);
pair<Vector2D,Vector2D> interceptPoints(Vector2D circle,double radius,Vector2D point2)
{
            pair<Vector2D,Vector2D> roots;
            double r_=radius;
            if(point2.x!=circle.x)
            {
            double m_=slope(circle.x,circle.y,point2.x,point2.y);
            double c_=circle.y-(m_*circle.x);
            roots.first.x=circle.x+sqrt(pow(r_,2)/(1+pow(m_,2)));
            roots.second.x=circle.x-sqrt(pow(r_,2)/(1+pow(m_,2)));
            roots.first.y=m_*roots.first.x+c_;
            roots.second.y=m_*roots.second.x+c_;
            }
            else
            {
                roots.first.x=circle.x;
                roots.second.x=circle.x;
                roots.first.y=circle.x+r_;
                roots.second.y=circle.x-r_;
            }



}

pair<Vector2D,Vector2D> interceptPoints(Vector2D circle,double radius,Vector2D point1,Vector2D point2)
{
            pair<Vector2D,Vector2D> roots;

            double r_=radius;
            if(point2.x!=point1.x)
            {
            double m_=slope(point1.x,point2.y,point2.x,point2.y);
            double c_=circle.y-(m_*circle.x);
            double a_ =(1+m_*m_);
            double b_ = -2*circle.x+2*m_*c_-2*circle.y*m_;
            double C_ = circle.x*circle.x+ pow((c_-circle.y),2) - r_*r_;

            double delta=b_*b_-4*a_*C_;

            if(delta>=0)
            {

            roots.first.x=(-b_+sqrt(delta))/(2*a_);
            roots.second.x=(-b_-sqrt(delta))/(2*a_);
            roots.first.y=m_* roots.first.x+c_;
            roots.second.y=m_* roots.second.y+c_;
            roots.first.x=circle.x+sqrt(pow(r_,2)/(1+pow(m_,2)));
            roots.second.x=circle.x-sqrt(pow(r_,2)/(1+pow(m_,2)));
            roots.first.y=m_*roots.first.x+c_;
            roots.second.y=m_*roots.second.x+c_;
            }


            }
            else if(r_*r_>=pow((circle.x-point1.x),2))
            {
                roots.first.x=point1.x;
                roots.second.x=point1.x;
                roots.first.y=circle.y+sqrt(r_*r_-pow((circle.x-point1.x),2) ) ;
                roots.second.y=circle.y-sqrt(r_*r_-pow((circle.x-point1.x),2) ) ;
            }
            return roots;



}
double getTackleProbe(PlayerAgent *agent);
double getGoalProbe(PlayerAgent* agent);
double getHoldProbe(PlayerAgent*);
void givePass(PlayerAgent * agent ,int passTaker,int passReceiver );
void givePass(PlayerAgent * agent ,int passTaker,int passReceiver,double ballspeed);
void givePass(PlayerAgent * agent ,int passTaker,int passReceiver,double ballspeed,Vector2D);

void giveThrough(PlayerAgent * agent,int unum,double approxDist);
void makeDribble(PlayerAgent * agent ,Vector2D point,int unum,double power);
bool passAttack(PlayerAgent * agent);
bool dribbleAttack(PlayerAgent * agent);
double getTackleProbe(PlayerAgent *agent)
{
    return 0.0;
}
double getGoalProbe(PlayerAgent* agent)
{
    return 0.0;
}
double getHoldProbe(PlayerAgent* agent)
{
    return 0.0;
}
void makeAttack(PlayerAgent* agent)
{
    if(passAttack(agent))
      cout<<"Passing... "<<endl;
    else
    if(dribbleAttack(agent))
    {
        cout<<"Dribbling..."<<endl;
    }

    else return;

}
pair<Vector2D,double> get_safe_point(Vector2D ball_point,int angle_start,int angle_end,int angle_thr,double sector_radius,vector<Vector2D> opponentslist,PlayerAgent * agent,int recr_steps)
{

    double discount_factor=0.9;
    vector<Vector2D> relative_opponentslist;
    pair<Vector2D,double> best_point;
    if(sector_radius>6)    
    best_point.second=0;
    else
    best_point.second=-1000000000;    
    best_point.first=ServerParam::i().theirTeamGoalPos();
    //cout<<"000000000000000000000000000000 step number"<<recr_steps<<endl;
    if(recr_steps==0)
    {
        best_point.second=0;
        return best_point;
    }    
    for(int i=angle_start;i<angle_end;i=i+angle_thr) // itterating clockwise
    {
        //cout<<"current  angle is"<<i<<endl;
        const Sector2D sector( ball_point,0, sector_radius,i ,i+angle_thr);
        if( agent->world().existOpponentIn( sector, 10, false ) )
           {
               // cout<<"opponent is at "<<i<<endl;
             continue;
           }
        Vector2D temp_point;
        temp_point.x=ball_point.x + sector_radius*cos(AngleDeg::DEG2RAD*(i+angle_thr/2));
        temp_point.y=ball_point.y + sector_radius*sin(AngleDeg::DEG2RAD*(i+angle_thr/2));
        double opponent_max_move=ball_point.dist(temp_point)/(agent->world().ball().vel().r())*ServerParam::i().defaultPlayerSpeedMax();        
        //cout<<"ok1   "<<opponentslist.size()<<endl;
        relative_opponentslist.clear();
        for(int j=0;j<opponentslist.size();j++)
        {
            Line2D temp_line(temp_point,opponentslist[i]);
            double temp_angle;
            if(temp_line.getB()!=0)
            temp_angle=atan(-1*temp_line.getA()/temp_line.getB());
            else
            temp_angle=AngleDeg::PI/2;    
            Vector2D temp_relative;
            temp_relative.x=opponentslist[i].x+opponent_max_move*cos(temp_angle+AngleDeg::PI);
            temp_relative.y=opponentslist[i].y+opponent_max_move*sin(temp_angle+AngleDeg::PI);
            relative_opponentslist.push_back(temp_relative);
        }
        //cout<<"ok1"<<endl;
        Vector2D close_opponent=opponentslist[0];
        for(int j=0;j<opponentslist.size();j++)
        {
            if(temp_point.dist(opponentslist[i])<temp_point.dist(close_opponent))
                close_opponent=opponentslist[i];
        }
        //cout<<"ok1"<<endl;
        double safety_value=temp_point.dist(close_opponent)
        +discount_factor*get_safe_point(temp_point,angle_start,angle_end,angle_thr,sector_radius,relative_opponentslist,agent,recr_steps-1).second;
        //cout<<"safety is equal"<<safety_value<<"  "<<i<<endl;
        if(best_point.second<safety_value)
        {
            best_point.second=safety_value;
            best_point.first=temp_point;
        }   

    }
    return best_point;
}
bool dribbleAttack(PlayerAgent * agent)
{


    int ball_step = agent->world().interceptTable()->opponentReachCycle();
    const WorldModel& wm=agent->world();

    const Vector2D ball_pos=wm.ball().pos();

    Vector2D desered_point=ServerParam::i().theirTeamGoalPos();
    PlayerPtrCont  dangerousPlayers;

    const PlayerPtrCont opp=wm.opponentsFromSelf();

    if(opp.empty())
    {
        makeDribble(agent,desered_point,wm.self().unum(),1);

        return true;
    }


    if(opp.size()>=1)
    {

        const PlayerObject *Nopponent=wm.getOpponentNearestToSelf(200);

        if(Nopponent->pos().dist(ball_pos)<=3)
        return false;
        int ahead_thr=6;
        Vector2D goal_post1= desered_point;
        goal_post1.y-=ServerParam::i().goalWidth()/2;
        Vector2D goal_post2= desered_point;
        goal_post2.y+=ServerParam::i().goalWidth()/2;
        AngleDeg goal_angle=(desered_point-wm.self().pos()).dir();
        AngleDeg post1_angle=(goal_post1-wm.self().pos()).dir();
        AngleDeg post2_angle=(goal_post2-wm.self().pos()).dir();
        Sector2D ahead1(wm.self().pos(),0.5, ahead_thr,goal_angle-90 ,goal_angle+90);
        
        vector<Vector2D> opponentslist;
        const PlayerPtrCont & opps = wm.opponentsFromSelf();
        for ( PlayerPtrCont::const_iterator it = opps.begin();it != opps.end();++it )
            opponentslist.push_back((*it)->pos());

        if(!wm.existOpponentIn(ahead1,10,false))
        {
            cout<<"far case is running ---"<<endl;
          //    desered_point.y=wm.self().pos().y;

              // makeDribble(agent,desered_point,wm.self().unum(),0.65);
              // return true;
                int recr_steps=3,angle_thr=20;
                double sector_radius=7;
                pair<Vector2D,double> safe_point=get_safe_point(wm.self().pos(),(int)post1_angle.degree(),(int)post2_angle.degree(),angle_thr,sector_radius,opponentslist,agent,recr_steps);
                
                makeDribble(agent,safe_point.first,wm.self().unum(),0.85);
                    return true;
                /*Sector2D upperSector(wm.self().pos(),0.5,100,post1_angle,goal_angle);
                Sector2D lowerSector(wm.self().pos(),0.5,100,goal_angle,post2_angle);

                //cout<<"Checking for all opponent ( "<<opp.size()<<" ) "<<endl;
                for ( PlayerPtrCont::const_iterator it = opp.begin();it != opp.end();++it )
                {
                  if( (upperSector.contains((*it)->pos()) )||(lowerSector.contains((*it)->pos()) ))
                    {
                        if((*it)->unum()>1)
                        {
                            dangerousPlayers.push_back((*it));
                //            cout<<"Player in a way "<<(*it)->unum()<<endl;
                        }

                    }
                    if(dangerousPlayers.size()==1)
                    break;

                }


                Vector2D goal_pos=ServerParam::i().theirTeamGoalPos();
                if(dangerousPlayers.empty())
                {
              //      cout<<"Safe Dribbling..."<<endl;
                    if(opp[0]->distFromSelf()<=10)
                        makeDribble(agent,goal_pos,wm.self().unum(),0.65);
                    else
                         makeDribble(agent,goal_pos,wm.self().unum(),0.65);
                   return true;
                }
                if(dangerousPlayers.size()==1)
                {

                    PlayerObject* oppP=dangerousPlayers[0];
                    if(lowerSector.contains((*oppP).pos()))
                    {
                        desered_point=goal_post1;
                    }
                    else
                    {
                        desered_point=goal_post2;
                    }
                //    cout<<"Ahead Dribbling..."<<endl;
                    makeDribble(agent,desered_point,wm.self().unum(),0.65);
                    return true;
                }
                */
        }
       else
        {
                int angle_thr=40,angle_start=-90,angle_end=90;
                int recr_steps=3;
                double sector_radius=3;
                //Sector2D ahead(wm.self().pos(),0, ahead_thr,-90 ,90);
                
                //cout<<"critical area entered..............."<<endl;
                pair<Vector2D,double> safe_point=get_safe_point(wm.self().pos(),angle_start,angle_end,angle_thr,sector_radius,opponentslist,agent,recr_steps);
               
                if(safe_point.second<0)
                    return false;
                else
                {
                   // cout<<wm.self().pos()<<" xxxxxxxxxxxxxx "<<safe_point.first<<endl;
                    Line2D temp_line(safe_point.first,wm.self().pos());
                    
                    double temp_slope=-1*temp_line.getA()/temp_line.getB();
                    printf("dircetion of dribbling is %lf\n",AngleDeg::RAD2DEG*atan(temp_slope));
                    //cout<<"dircetion of dribbling is"<<AngleDeg::RAD2DEG*atan(temp_slope)<<endl;
                    makeDribble(agent,safe_point.first,wm.self().unum(),0.65);
                    return true;
                }
                /*
                const PlayerPtrCont & opps = wm.opponentsFromSelf();
                for ( PlayerPtrCont::const_iterator it = opps.begin();it != opps.end();++it )
                {
                     if(ahead.contains((*it)->pos()))
                     {
                        Nopponent=(*it);
                      break;
                     }
                }

                cout<<"Colsest Player is "<<Nopponent->unum()<<endl;
                int dist_thr=6;
                int angle_thr=15;
                AngleDeg target_angle=(Nopponent->pos()-wm.self().pos()).dir();
                bool notFound=true;
                //Sector2D findSector;// (wm.self().pos(),0, dist_thr,target_angle,target_angle  );
                AngleDeg farAngle=target_angle;
                double minDist=0;
                int temp_dir=-90;
                for(int i=-90;i<=(90-angle_thr);i+=angle_thr)
                {

                    const Sector2D sector( wm.self().pos(),0, dist_thr,i ,i+angle_thr);
                    AngleDeg newAngle=i+angle_thr/2;
                    cout<<sector.center()<<" NewPoint "<<newAngle<<" "<<i<<endl;
                     const Sector2D sector( wm.self().pos(),0, dist_thr,i ,i+angle_thr);
                    if( !wm.existOpponentIn( sector, 10, false ) )
                    {
                        
                      
                        cout<<abs(newAngle.degree()-target_angle.degree())<<" "<<abs(farAngle.degree()-target_angle.degree());
                        if(abs(newAngle.degree()-target_angle.degree())>abs(farAngle.degree()-target_angle.degree()))
                        {

                            notFound=false;
                            farAngle=newAngle;
                            temp_dir=i;
                        }
                   

                    }
                    else
                    cout<<"opponent is at sector of angle"<<i<<endl;

                }


                if(notFound==true)
                return false;
                else
                {
                        cout<<"going at atatatatat"<<farAngle<<endl;                    


                      cout<<Nopponent->unum()<<" Moving ..."<<" "<<temp_dir<<endl;
                
                      Line2D direction=Line2D(wm.ball().pos(),farAngle);
                      Vector2D goal_point=ServerParam::i().theirTeamGoalPos();
                      Vector2D goal_point2=goal_point;
                      goal_point2.y=100;
                      Line2D goalLine=Line2D(goal_point,goal_point2);
                      cout<<direction.intersection(goalLine);
                      makeDribble(agent,direction.intersection(goalLine),wm.self().unum(),0.65);
                      return true;
                }

            */
        }


   }
    return false;

}

Vector2D best_catching_point(Vector2D ball_pos,Vector2D player_pos,int thr,double ratio,double vel1,double vel2)
{
  Vector2D point,point2;
  Line2D L1(ball_pos,thr);
  Line2D direct_line(ball_pos,player_pos);
// double t_1;
//   if(L1.getB()!=0)
//    t_1 = atan(-L1.getA()/L1.getB());
//    else
//     t_1=AngleDeg::PI/2;

//   double t_2;
//   if(direct_line.getB()!=0)
//   t_2= atan(-direct_line.getA()/direct_line.getB());
//   else
//   t_2=AngleDeg::PI/2;


//   AngleDeg alpha((t_1-t_2)*AngleDeg::RAD2DEG);
  AngleDeg alpha(thr-(player_pos-ball_pos).dir().degree());

  
  AngleDeg beta(AngleDeg::RAD2DEG*asin(ratio*sin(alpha.radian()) ) );
  AngleDeg beta2(180-beta.degree());
  double d=ball_pos.dist(player_pos);
  if((vel1*cos(alpha.radian()) - vel2*cos(beta.radian()))==0.0)
   {
        point.x=10000;
        point.y=10000;
        
   } 
   else
   {
        double dist_to_inter=vel1*d/(vel1*cos(alpha.radian()) - vel2*cos(beta.radian()));
      point.x=ball_pos.x+dist_to_inter*cos(thr*AngleDeg::DEG2RAD);
    point.y=ball_pos.y+dist_to_inter*sin(thr*AngleDeg::DEG2RAD);
  
   }
  

  
  if((vel1*cos(alpha.radian()) - vel2*cos(beta2.radian()))==0.0)
   {
        point2.x=10000;
        point2.y=10000;
        
   }
   else
   {
        double dist_to_inter=vel1*d/(vel1*cos(alpha.radian()) - vel2*cos(beta2.radian()));
        point2.x=ball_pos.x+dist_to_inter*cos(thr*AngleDeg::DEG2RAD);
        point2.y=ball_pos.y+dist_to_inter*sin(thr*AngleDeg::DEG2RAD);
  
   } 
  
  //Line2D L2(player_pos,beta);

// passing point for current player
  //if(L1.getA()/L1.getB() == L2.getA()/L2.getB())
   // cout<<"Alpha "<<alpha.degree()<<endl;
   // cout<<"Beta "<<beta.degree()<<endl;
   // cout<<"Beta2 "<<beta2.degree()<<endl;

   

  //point=L1.intersection(L2);
   if(ball_pos.dist(point)<ball_pos.dist(point2))
  return point;
    else
        return point2;
}

class POINTS
{
public:
    double dis;
    int player_num;
    Vector2D tackle_point; 
};
struct Comp{
        bool operator()(const POINTS& a, const POINTS& b){
            return a.dis<b.dis;
        }
    };
bool passAttack(PlayerAgent* agent)
{
  const double ratio=ServerParam::i().ballSpeedMax()/(ServerParam::i().defaultPlayerSpeedMax());
  const double vel1=ServerParam::i().ballSpeedMax();
  const double vel2=(ServerParam::i().defaultPlayerSpeedMax());
  const WorldModel& wm=agent->world();

  //const PlayerPtrCont & opps = wm.opponentsFromSelf();
  //const PlayerPtrCont & ours = wm.teammatesFromSelf();
  const PlayerPtrCont & opps = (const PlayerPtrCont &)wm.theirPlayers();
    const PlayerPtrCont & ours = (const PlayerPtrCont &)wm.ourPlayers();
  AngleDeg angle_thr(10);
  Vector2D ball_pos=wm.ball().pos();

  PlayerObject *bestPlayer=NULL;
  // if(opps.size()==0)
  // {
  //     bestPlayer=passPriority.top().second;
  //     givePass(agent,agent->world().self().unum(),bestPlayer->unum());
  //     return true;
  // }

  bool canPass=false;
  Vector2D pass_point;
  int f_lenght=ServerParam::i().pitchHalfLength();
  int f_width=ServerParam::i().pitchHalfWidth();
  cout<<f_lenght<<" pitch  "<<f_width<<endl;
  cout<<"ballllll"<<ball_pos<<endl;
  // - +, + -
  Rect2D field(Vector2D(-f_lenght,-f_width), Vector2D(f_lenght,f_width) ); 

    
    // typedef pair<int,Vector2D*> PLAYER;
    // //typedef pair<int,Vector2D> PLAYER11;
    // //priority_queue<PLAYER11> passPoints11;
    // typedef pair<double, PLAYER > POINTS;
    // typedef pair<double, Vector2D* > OPPO;
    priority_queue<POINTS,vector<POINTS>,Comp> passPoints;


for(int i=0;i<180;i+=angle_thr.degree())
  {
    priority_queue<POINTS,vector<POINTS>,Comp> red_points;
    for ( PlayerPtrCont::const_iterator it = opps.begin();it != opps.end();++it )
    {


        Vector2D red_point=best_catching_point(ball_pos,(*it)->pos(),i,ratio,vel1,vel2);
       // cout<<"player  "<<(*it)->unum()<<"tackles at   "<<red_point<<"present at "<<(*it)->pos()<<endl;
        //if(field.contains( red_point) )
        if(red_point.x<f_lenght && red_point.x>-1*f_lenght &&red_point.y<f_width &&red_point.y>-1*f_width )
        {
            //cout<<" inside field"<<endl;
        double distance_ball=red_point.dist(ball_pos);
        POINTS current;
        current.dis=-distance_ball;
        current.tackle_point=red_point;
        current.player_num=(*it)->unum();
        red_points.push(current);
        }

        // else
        // {
        //    // cout<<" outside field"<<endl;
        //     ;
        // }
      //   if(!red_points.empty())
      // cout<<"Tackle Point is "<<*(red_points.top().second)<<" and "<<i<<endl;

    }

    // if(!red_points.empty())
    // {
    //     cout<<"for direction  "<<i<<"  opponent can tackle  "<<*(red_points.top().second)<<endl;
    // }
    // else
    // {
    //     cout<<"for direction  "<<i<<" 000000000000no opponent can tackle"<<endl;
    // }
    for ( PlayerPtrCont::const_iterator it = ours.begin();it != ours.end();++it )
    {
          Vector2D oppGoalpost=ServerParam::i().theirTeamGoalPos();
          Vector2D closest_opp_point;
          if(!red_points.empty())
        closest_opp_point  =red_points.top().tackle_point;

          Vector2D blue_point=best_catching_point(ball_pos,(*it)->pos(),i,ratio,vel1,vel2);
          // cout<<"player  "<<(*it)->unum()<<"tackles at   "<<blue_point<<"present at "<<(*it)->pos()<<endl;
          // cout<<"closeset opponent"<<(closest_opp_point)<<"   "<<red_points.top().player_num<<endl;
          // if(field.contains(blue_point))
          // cout<<(*it)->unum()<<" is catching point"<<blue_point<<endl;

          if( (red_points.empty()||(  blue_point.dist(ball_pos)<closest_opp_point.dist(ball_pos)))
           && blue_point.x<f_lenght && blue_point.x>-1*f_lenght &&blue_point.y<f_width &&blue_point.y>-1*f_width )
          {
            if( blue_point.x<f_lenght && blue_point.x>-1*f_lenght && blue_point.y<f_width && blue_point.y>-1*f_width )
              {
            double distance_goal=blue_point.dist(oppGoalpost);
            POINTS current;
            current.dis=-distance_goal;
            current.player_num=(*it)->unum();
            current.tackle_point=blue_point;
            passPoints.push(current);
          }
          }
    }
  }
  if(!passPoints.empty())
  {


        AngleDeg a=((passPoints.top().tackle_point)-ball_pos).dir();
        cout<<"Best Player of  is "<<passPoints.top().player_num<<" Point "<<((passPoints.top().tackle_point))<<" "<<a<<endl;
        givePass(agent,agent->world().self().unum(),passPoints.top().player_num,ServerParam::i().ballSpeedMax()*0.7,((passPoints.top().tackle_point)));
        return true;
  }

  return false;


}



/*
bool passAttack(PlayerAgent * agent)
{


    const double max_speed_ballplayer=ServerParam::i().ballSpeedMax()/ServerParam::i().defaultPlayerSpeedMax();

    const WorldModel& wm=agent->world();
    const Vector2D ball_pos=wm.ball().pos();
    Vector2D pass_point;
    const AbstractPlayerCont & opps = wm.theirPlayers();
    const AbstractPlayerCont & ours = wm.ourPlayers();
    typedef pair<double,const AbstractPlayerObject *> PLAYER;


    Vector2D nearest_pos(0,0);
    const AbstractPlayerObject *bestPlayer=NULL;
    bool canPass=false;
    //cout<<endl;
    priority_queue<PLAYER> passPriority;
    vector<Line2D> oppCaptured;
    vector<const AbstractPlayerObject *> oppP;

    oppCaptured.push_back(Line2D(0,0,0));
    oppP.push_back(0);

     for ( AbstractPlayerCont::const_iterator it = opps.begin();it != opps.end();++it )
        {

            double distf_ball=(double)(*it)->pos().dist(wm.ball().pos());

            Line2D connect=Line2D(ball_pos,(*it)->pos());
            Vector2D interceptPoint=(ball_pos + (*it)->pos() ) /((1+max_speed_ballplayer)/max_speed_ballplayer);

            oppCaptured.push_back(Line2D(connect.getB(),-connect.getA(),connect.getB()*interceptPoint.y-connect.getB()*interceptPoint.x ));
            oppP.push_back((*it));
        }

    for ( AbstractPlayerCont::const_iterator it = ours.begin();it != ours.end();++it )
        {
            if((*it)->unum()==wm.self().unum())
                continue;
            double distf_goal=(*it)->pos().dist(ServerParam::i().theirTeamGoalPos());
            PLAYER current;
            current.first=-distf_goal;
            current.second=(*it);
            passPriority.push(current);
        }


    while  (!passPriority.empty() && canPass==false)
    {


         const AbstractPlayerObject* current=passPriority.top().second;
         const double pitchHWidth=ServerParam::i().pitchHalfWidth();
         const double pitchHLength=ServerParam::i().pitchHalfLength();
         Rect2D pitch=Rect2D(Vector2D(-pitchHLength,-pitchHWidth),Vector2D(pitchHLength,pitchHWidth));
         Vector2D getPoint=(ball_pos + current->pos() ) /((1+max_speed_ballplayer)/max_speed_ballplayer);
         Line2D connect=Line2D(ball_pos,current->pos());
         Line2D curr_line=Line2D(connect.getB(),-connect.getA(),connect.getA()*getPoint.y-connect.getB()*getPoint.x );

         Vector2D point1;
         point1=curr_line.intersection(Line2D(Vector2D(0,-pitchHWidth),Vector2D(1,-pitchHWidth) ));
         if(!pitch.contains(point1))
         {point1=curr_line.intersection(Line2D( Vector2D(pitchHLength,0),Vector2D(pitchHLength,1) ));
         if(!pitch.contains(point1))
            point1=curr_line.intersection(Line2D( Vector2D(-pitchHLength,0),Vector2D(-pitchHLength,1) ));
         }
        Vector2D point2;
         point2=curr_line.intersection(Line2D(Vector2D(0,pitchHWidth),Vector2D(1,pitchHWidth) ));
        if(!pitch.contains(point2))
         {point2=curr_line.intersection(Line2D( Vector2D(pitchHLength,0),Vector2D(pitchHLength,1) ));
         if(!pitch.contains(point2))
            point2=curr_line.intersection(Line2D( Vector2D(-pitchHLength,0),Vector2D(-pitchHLength,1) ));
         }



         Segment2D remains=Segment2D(point1,point2);
         for(int i=12;i>0;i--)
        {

        Vector2D opp_pos;
        Vector2D cur_pos=current->pos();
        Line2D opp_line=oppCaptured[i];

        Vector2D inP=curr_line.intersection(opp_line),curr_line;
        if(!pitch.contains(inP))
        {
            if( (opp_line.getA()*curr_line.x+opp_line.getB()*curr_line.y+opp_line.getC()>=0 )

            )
            {
                canPass=false;

                cout<<"Can not give a pass to player "<<current->unum()<<endl;
                 break;
            }
            else
                continue;
        }



       if(inP.y>=cur_pos.y)

       {       remains=Segment2D(inP,remains.terminal());

       }
       else
       {remains=Segment2D(remains.origin(),inP);

       }


       if(remains.length()<3)
       {
        canPass=false;
        break;
        cout<<"\rCan not give a pass to player "<<current->unum()<<endl;
       }

        if(i==11)
        {
            canPass=true;
            bestPlayer=current;
            pass_point=(remains.origin()+remains.terminal())/2;
            cout<<remains.origin()<< " "<<remains.terminal()<<endl;
            break;
        }

         }
          if(canPass==false)
        passPriority.pop();
     }
    if(canPass)
    {
         cout<<endl<<"\rGiving Best Pass to Player "<<bestPlayer->unum()<<endl;
         double distance=(double) pass_point.dist(agent->world().self().pos());

        givePass(agent,agent->world().self().unum(),bestPlayer->unum(),ServerParam::i().ballSpeedMax()*(distance/45),pass_point);
        return true;
    //    giveThrough(agent,bestPlayer->unum(),nearest_pos);
    }

    //cout<<endl;
    return false;
}*/
/*bool passAttack(PlayerAgent * agent)
{
    const double max_speed_ballplayer=ServerParam::i().ballSpeedMax()/ServerParam::i().defaultPlayerSpeedMax();
    const WorldModel& wm=agent->world();
    const PlayerPtrCont & opps = wm.opponentsFromSelf();
    const PlayerPtrCont & ours = wm.teammatesFromSelf();
    typedef pair<double,PlayerObject *> PLAYER;
    typedef pair<double,PlayerObject *> Probe_Circle;
    priority_queue<PLAYER> passPriority;
    vector<Probe_Circle> oppCapturedCircle;
    vector<Probe_Circle> ourCapturedCircle;
    Probe_Circle _NULL;
    oppCapturedCircle.push_back(_NULL);
    ourCapturedCircle.push_back(_NULL);
    for ( PlayerPtrCont::const_iterator it = opps.begin();it != opps.end();++it )
    {
        double distancef_ball=(double)(*it)->pos().dist(agent->world().ball().pos());
        Probe_Circle curerntArea;
        curerntArea.first=distancef_ball/(1+max_speed_ballplayer);
        curerntArea.second=(*it);
        oppCapturedCircle.push_back(curerntArea);
    }
    for ( PlayerPtrCont::const_iterator it = ours.begin();it != ours.end();++it )
    {
        Vector2D oppGoalpost(0.0,0.0 );
        double distancet_goal=(*it)->pos().dist(oppGoalpost);
        PLAYER current;
        current.first=-distancet_goal;
        current.second=(*it);
        passPriority.push(current);
        double distancef_ball=(double)(*it)->pos().dist(agent->world().ball().pos());
        Probe_Circle curerntArea;
        curerntArea.first=distancef_ball/(1+max_speed_ballplayer);
        curerntArea.second=(*it);
        ourCapturedCircle.push_back(curerntArea);
    }
    PlayerObject *bestPlayer=NULL;
    bool canPass=false;
    Vector2D pass_point;
    while(!passPriority.empty() && !bestPlayer)
    {
        cout<<"Checking pass of Player "<<passPriority.top().second->unum()<<endl;
        PlayerObject* current=passPriority.top().second;
        for(int i=1;i<12;i++)
        {
            Vector2D opp_pos;
            Vector2D cur_pos=current->pos();
            opp_pos=oppCapturedCircle[i].second->pos();
            if(pow((opp_pos.x-cur_pos.x),2)+pow((opp_pos.y-cur_pos.y),2)<=pow(oppCapturedCircle[i].first,2))
            {
                break;
                cout<<"Player "<<bestPlayer->unum()<<"Can be tacked..."<<endl;
            }
            if(i==11)
            {
                bestPlayer=current ;
                if(bestPlayer)
                canPass=true;

            }
        }
        if(canPass==false)
        passPriority.pop();
    }
    if(canPass)
    {
    cout<<"Giving Best Pass to Player "<<bestPlayer->unum()<<endl;
    double distance=wm.ball().pos().dist(bestPlayer->pos());
    givePass(agent,agent->world().self().unum(),bestPlayer->unum(),ServerParam::i().ballSpeedMax()*(distance/45));
    }
}*/

Formation::Ptr  getFormation()
{

   Formation::Ptr t_formation;
   const string & file_path="formations-dt/"+ATTACK_FORMATION_CONF;
   ifstream fin( file_path.c_str() );
   if ( ! fin.is_open() )
    {
        std::cerr << __FILE__ << ':' << __LINE__ << ':'<< " ***ERROR*** failed to open file [" << file_path << "]"<< std::endl;
        return t_formation;
    }

     t_formation=Formation::create(string("DelaunayTriangulation"));
    if ( ! t_formation->read( fin ) )
    {
       std::cerr << __FILE__ << ':' << __LINE__ << ':'<< " ***ERROR*** failed to open file [" << file_path << "]"<< std::endl;
        t_formation.reset();
        return t_formation;
    }

    return t_formation;
}


void updatePositionOf(PlayerAgent* agent,Formation::Ptr formation)
{
     int unum=agent->world().self().unum();
    int ball_step = 0;
    if ( agent->world().gameMode().type() == GameMode::PlayOn || agent->world().gameMode().type() == GameMode::GoalKick_ )
    {
        ball_step = std::min( 1000, agent->world().interceptTable()->teammateReachCycle() );
        ball_step = std::min( ball_step, agent->world().interceptTable()->opponentReachCycle() );
        ball_step = std::min( ball_step, agent->world().interceptTable()->selfReachCycle() );
    }

    Vector2D ball_pos = agent->world().ball().inertiaPoint( ball_step);
    p_positions.clear();
     formation::SampleDataSet::Ptr s_data=formation->samples();
     if(!s_data || !formation)
     {
        std::cerr << __FILE__ << ':' << __LINE__ << ':'<<"SampleDataSet or Formation is NULL"<<endl;
        return;
     }
    formation->getPositions(ball_pos, p_positions );


    if(p_positions[unum-1].x<=50 && p_positions[unum-1].x>=-50 )
    {  double maxDashPower=ServerParam::i().maxDashPower();
    #ifdef MYDEBUG
    cout<<"positions of "<<unum<<" is "<<p_positions[unum-1]<<endl;
    #endif
       Body_GoToPoint( p_positions[unum-1], 1,maxDashPower ).execute( agent );
     }



    //cout<<"positions of ball for "<<unum<<" is "<<formation->samples()>ball_<<endl;

}



double minReachableSpeed(PlayerAgent * agent,Vector2D receiver_pos)
{

      double distance=(double) receiver_pos.dist(agent->world().self().pos());
      return ServerParam::i().ballSpeedMax()*(distance/45);
}
void givePass(PlayerAgent * agent ,int passTaker,int passReceiver)
{
    const AbstractPlayerObject * receiver = agent->world().ourPlayer(passReceiver);
            if(!receiver)return;
    const Vector2D receiver_pos =receiver->pos();
    double ballspeed=minReachableSpeed(agent,receiver_pos)*1.75;

    givePass(agent,passTaker,passReceiver,ballspeed);
}
void  givePass(PlayerAgent * agent ,int passTaker,int passReceiver,double ballspeed,Vector2D point)
{
const AbstractPlayerObject * receiver = agent->world().ourPlayer(passReceiver);
if(!receiver)return;
  const Vector2D receiver_pos =point;

    if(agent->world().self().unum()==passTaker)
        {

         if( agent->world().self().isKickable())
        {

           double distance=(double) receiver_pos.dist(agent->world().self().pos());
           //double ballspeed=ServerParam::i().ballSpeedMax()/(distance/45); //sqrt(pow(agent->world().ball().vel().x,2)+pow(agent->world().ball().vel().y,2));

           Body_TurnToPoint( receiver_pos ).execute( agent );
           Body_KickOneStep kick(receiver_pos,ballspeed);
           kick.execute( agent);
           Vector2D ball_vel( 0.0, 0.0 );
           if ( ! agent->effector().queuedNextBallKickable() )
           ball_vel = agent->effector().queuedNextBallVel();
           ((SamplePlayer*)agent)->lastRole="Passer";
           agent->addSayMessage(new PassMessage( passReceiver,receiver_pos ,agent->effector().queuedNextBallPos(),ball_vel));

           #ifdef MYDEBUG
           cout<<"Distance :"<<distance<<endl;
           cout<<PassMessage( passReceiver,point ,agent->effector().queuedNextBallPos(),ball_vel).header()<<endl<<endl;
            #endif

        }
        }


}
void givePass(PlayerAgent * agent ,int passTaker,int passReceiver,double ballspeed)
{
//  const double dash_power = Strategy::get_normal_dash_power( agent->world() );
    const AbstractPlayerObject * receiver = agent->world().ourPlayer(passReceiver);
            if(!receiver)return;
    givePass(agent,passTaker,passReceiver,ballspeed,receiver->pos());

}
void makeDribble(PlayerAgent * agent ,Vector2D point,int unum,double power)
{
     Body_Dribble( point,
                      1.0,
                      ServerParam::i().maxDashPower(),
                      1
                      ).execute( agent );
     return;

        if(!agent)
        return;
        if(agent->world().self().unum()==unum)
        {
            bool kickable = agent->world().self().isKickable();
            if(kickable)
            {
             Body_TurnToPoint( point ).execute( agent );
             Body_KickOneStep(point,power ).execute( agent);
            }
            else
            Body_GoToPoint( agent->world().ball().pos(), power, 1).execute( agent );


        }

}

void giveThrough(PlayerAgent * agent,int unum,double approxDist)
{
    const WorldModel & wm = agent->world();
    const PlayerObject * teammate=wm.getTeammateNearestToSelf(1);
   if(!teammate)
       return;
    if( !wm.self().isKickable())
    return;
      //   const vector<AudioMemory::Pass>& M_pass=agent->world().audioMemory().pass();


            Vector2D pass_point(0.0,0.0);
           pass_point=teammate->pos()+Vector2D(approxDist,0.0);
            double ballspeed=ServerParam::i().ballSpeedMax();
           double distance=(double) pass_point.dist(agent->world().self().pos());
           Body_TurnToPoint(  pass_point ).execute( agent );
           Body_KickOneStep(  pass_point,ballspeed*(distance/30)).execute( agent);
           Vector2D ball_vel( 0.0, 0.0 );
           if ( ! agent->effector().queuedNextBallKickable() )
           ball_vel = agent->effector().queuedNextBallVel();
           const ActionEffector& M_effector=agent->effector();
           const PassMessage* mes=new PassMessage( teammate->unum(),pass_point ,M_effector.queuedNextBallPos(),ball_vel);
           agent->addSayMessage(mes);
           ((SamplePlayer*)agent)->lastRole="Passer";



}
/*
double getDashPower(PlayerAgent* agent,bool max=false)
{
    const WorldModel & wm = agent->world();
    const int self_reach = wm.interceptTable()->selfReachCycle();
    const int mate_reach = wm.interceptTable()->teammateReachCycle();
    const int opp_reach = wm.interceptTable()->opponentReachCycle();

}
*/
void collectPass(PlayerAgent * agent)
{

    const vector<AudioMemory::Pass>& M_pass=agent->world().audioMemory().pass();
    const WorldModel & wm = agent->world();

    if (     M_pass.empty()
        ||   M_pass.front().receiver_ != wm.self().unum()
         || ((SamplePlayer*)agent)->lastRole=="Passer")
    {
        return ;
    }

    const double dash_power = (wm.self().stamina()/ServerParam::i().staminaMax())*ServerParam::i().maxDashPower();//Strategy::get_normal_dash_power( agent->world() );
    double distance=wm.ball().pos().dist(agent->world().self().pos());
    Vector2D collect_pos(0.0,0.0);
    const int reach_ball_step= wm.interceptTable()->selfReachCycle();
    collect_pos=wm.ball().inertiaPoint( reach_ball_step );


    Body_GoToPoint(collect_pos, 1, dash_power).execute( agent );

    ((SamplePlayer*)agent)->lastRole="Receiver";
    if( wm.self().isKickable())
    {
         ((SamplePlayer*)agent)->lastRole="Normal";

    }
}
void runThrough(PlayerAgent * agent)
{

   collectPass(agent);
}
SamplePlayer::SamplePlayer()
    : PlayerAgent(),
      M_communication(),
      M_field_evaluator( createFieldEvaluator() ),
      M_action_generator( createActionGenerator() )
{
    boost::shared_ptr< AudioMemory > audio_memory( new AudioMemory );

    M_worldmodel.setAudioMemory( audio_memory );

    //
    // set communication message parser
    //
    addSayMessageParser( SayMessageParser::Ptr( new BallMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new PassMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new InterceptMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new GoalieMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new GoalieAndPlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new OffsideLineMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new DefenseLineMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new WaitRequestMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new PassRequestMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new DribbleMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new BallGoalieMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new OnePlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new TwoPlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new ThreePlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new SelfMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new TeammateMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new OpponentMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new BallPlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new StaminaMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new RecoveryMessageParser( audio_memory ) ) );

    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 9 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 8 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 7 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 6 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 5 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 4 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 3 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 2 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 1 >( audio_memory ) ) );

    //
    // set freeform message parser
    //
    setFreeformParser( FreeformParser::Ptr( new FreeformParser( M_worldmodel ) ) );

    //
    // set action generators
    //
    // M_action_generators.push_back( ActionGenerator::Ptr( new PassGenerator() ) );

    //
    // set communication planner
    //
    M_communication = Communication::Ptr( new SampleCommunication() );
}

/*-------------------------------------------------------------------*/
/*!

 */
SamplePlayer::~SamplePlayer()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
bool
SamplePlayer::initImpl( CmdLineParser & cmd_parser )
{
    bool result = PlayerAgent::initImpl( cmd_parser );

    // read additional options
    result &= Strategy::instance().init( cmd_parser );

    rcsc::ParamMap my_params( "Additional options" );
#if 0
    std::string param_file_path = "params";
    param_map.add()
        ( "param-file", "", &param_file_path, "specified parameter file" );
#endif

    cmd_parser.parse( my_params );

    if ( cmd_parser.count( "help" ) > 0 )
    {
        my_params.printHelp( std::cout );
        return false;
    }

    if ( cmd_parser.failed() )
    {
        std::cerr << "player: ***WARNING*** detected unsuppprted options: ";
        cmd_parser.print( std::cerr );
        std::cerr << std::endl;
    }

    if ( ! result )
    {
        return false;
    }

    if ( ! Strategy::instance().read( config().configDir() ) )
    {
        std::cerr << "***ERROR*** Failed to read team strategy." << std::endl;
        return false;
    }

    if ( KickTable::instance().read( config().configDir() + "/kick-table" ) )
    {
        std::cerr << "Loaded the kick table: ["
                  << config().configDir() << "/kick-table]"
                  << std::endl;
    }

    return true;
}

/*-------------------------------------------------------------------*/
/*!
  main decision
  virtual method in super class
*/
void
SamplePlayer::actionImpl()
{
    //
    // update strategy and analyzer
    //
    Strategy::instance().update( world() );
    FieldAnalyzer::instance().update( world() );

    //
    // prepare action chain
    //
    M_field_evaluator = createFieldEvaluator();
    M_action_generator = createActionGenerator();

    debugClient().addLine(Vector2D(0,0),Vector2D(52.5,0));

    ActionChainHolder::instance().setFieldEvaluator( M_field_evaluator );
    ActionChainHolder::instance().setActionGenerator( M_action_generator );

    //
    // special situations (tackle, objects accuracy, intention...)
    //
    const WorldModel & wm = this->world();

    if ( doPreprocess() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": preprocess done" );
        return;
    }

    //
    // update action chain
    //
    ActionChainHolder::instance().update( world() );

    //
    // create current role
    //
    std::string role_name;
    SoccerRole::Ptr role_ptr;
    {
        role_ptr = Strategy::i().createRole( world().self().unum(), world() );
        role_name = Strategy::i().getRoleName( world().self().unum(), world() );

        if ( ! role_ptr )
        {
            std::cerr << config().teamName() << ": "
                      << world().self().unum()
                      << " Error. Role is not registerd.\nExit ..."
                      << std::endl;
            M_client->setServerAlive( false );
            return;
        }
    }

    //
    // override execute if role accept
    //
    if ( role_ptr->acceptExecution( world() ) )
    {
        if(role_name=="Sample"){
            executeSampleRole(this);
            return;
        }
        role_ptr->execute( this );
        return;
    }

    //
    // play_on mode
    //
    if ( world().gameMode().type() == GameMode::PlayOn )
    {
        if(role_name=="Sample"){
            executeSampleRole(this);
            return;
        }
        role_ptr->execute( this );
        return;
    }

    if ( world().gameMode().type() == GameMode::KickOff_)
    {
        o_formation=getFormation();
        mpIntransit = false;
        if(wm.self().unum()==10){
            mpIntransit = true;
            mpTarget = wm.ball().pos();


        }

        bool kickable = this->world().self().isKickable();
        if ( this->world().existKickableTeammate()
             && this->world().teammatesFromBall().front()->distFromBall()
             < this->world().ball().distFromSelf() )
        {
            kickable = false;
        }

      /// givePass(this,wm.self().unum(),1);//world().teammatesFromSelf().front()->unum());
       //    makeDribble(this,Vector2D(-10,0),world().self().unum(),1);
        if ( kickable )
        {
          //   giveThrough(this,wm.self().unum()ServerParam::i().defaultPlayerSpeedMax(),10);
            makeAttack(this);
            //cout<<ServerParam::i().ballSpeedMax()<<" "<<ServerParam::i().defaultPlayerSpeedMax()<<endl;
         /*  cout<<best_catching_point(Vector2D(-10.7,-12.12),Vector2D(-16.59,-11.95),0,
                ServerParam::i().ballSpeedMax()/ServerParam::i().defaultPlayerSpeedMax(),
                ServerParam::i().ballSpeedMax(),
                ServerParam::i().defaultPlayerSpeedMax())<<endl;*/
                // cout<<(Vector2D(-10.7,-12.12)-Vector2D(-16.59,-11.95)).dir().degree()<<endl;
        
         //  givePass(this,wm.self().unum(),2);
            //Bhv_BasicMove().execute(this);
          //  Bhv_BasicOffensiveKick().execute(this);
        //    if(!PassToBestPlayer( this )){
        //        Body_HoldBall().execute( this );
            //    doKick( this);
                //Bhv_BasicOffensiveKick().execute(this);
               // PassToBestPlayer(this);
            //f}
        }
        else
        {
            Bhv_BasicMove().execute(this);
        }
        return;
    }

    //
    // penalty kick mode
    //
    if ( world().gameMode().isPenaltyKickMode() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": penalty kick" );
        Bhv_PenaltyKick().execute( this );
        return;
    }

    //
    // other set play mode
    //

    if(role_name=="Sample"){
        executeSampleRole(this);
        return;
    }
}



/*--------------------------------------------------------------------------*/

bool
SamplePlayer::SampleDribble( PlayerAgent * agent )
{
    const WorldModel & wm = agent->world();

    const PlayerPtrCont & opps = wm.opponentsFromSelf();
    const PlayerObject * nearest_opp
        = ( opps.empty()
            ? static_cast< PlayerObject * >( 0 )
            : opps.front() );
    const double nearest_opp_dist = ( nearest_opp
                                      ? nearest_opp->distFromSelf()
                                      : 1000.0 );
    const Vector2D nearest_opp_pos = ( nearest_opp
                                       ? nearest_opp->pos()
                                       : Vector2D( -1000.0, 0.0 ) );

    Vector2D pass_point;

    if ( Body_Pass::get_best_pass( wm, &pass_point, NULL, NULL ) )
    {
        if ( pass_point.x > wm.self().pos().x - 1.0 )
        {
            bool safety = true;
            const PlayerPtrCont::const_iterator opps_end = opps.end();
            for ( PlayerPtrCont::const_iterator it = opps.begin();
                  it != opps_end;
                  ++it )
            {
                if ( (*it)->pos().dist( pass_point ) < 4.0 )
                {
                    safety = false;
                }
            }

            if ( safety )
            {
                Body_Pass().execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true;
            }
        }
    }

    if ( nearest_opp_dist < 3.0 )
    {
        if ( Body_Pass().execute( agent ) )
        {
            agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
            return true;
        }
    }

    // dribble to my body dir
    if ( nearest_opp_dist < 5.0
         && nearest_opp_dist > ( ServerParam::i().tackleDist()
                                 + ServerParam::i().defaultPlayerSpeedMax() * 1.5 )
         && wm.self().body().abs() < 70.0 )
    {
        const Vector2D body_dir_drib_target
            = wm.self().pos()
            + Vector2D::polar2vector(5.0, wm.self().body());

        int max_dir_count = 0;
        wm.dirRangeCount( wm.self().body(), 20.0, &max_dir_count, NULL, NULL );

        if ( body_dir_drib_target.x < ServerParam::i().pitchHalfLength() - 1.0
             && body_dir_drib_target.absY() < ServerParam::i().pitchHalfWidth() - 1.0
             && max_dir_count < 3
             )
        {
            // check opponents
            // 10m, +-30 degree
            const Sector2D sector( wm.self().pos(),
                                   0.5, 10.0,
                                   wm.self().body() - 30.0,
                                   wm.self().body() + 30.0 );
            // opponent check with goalie
            if ( ! wm.existOpponentIn( sector, 10, true ) )
            {
                Body_Dribble( body_dir_drib_target,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true;
            }
        }
    }

    Vector2D drib_target( 50.0, wm.self().pos().absY() );
    if ( drib_target.y < 20.0 ) drib_target.y = 20.0;
    if ( drib_target.y > 29.0 ) drib_target.y = 27.0;
    if ( wm.self().pos().y < 0.0 ) drib_target.y *= -1.0;
    const AngleDeg drib_angle = ( drib_target - wm.self().pos() ).th();

    // opponent is behind of me
    if ( nearest_opp_pos.x < wm.self().pos().x + 1.0 )
    {
        // check opponents
        // 15m, +-30 degree
        const Sector2D sector( wm.self().pos(),
                               0.5, 15.0,
                               drib_angle - 30.0,
                               drib_angle + 30.0 );
        // opponent check with goalie
        if ( ! wm.existOpponentIn( sector, 10, true ) )
        {
            const int max_dash_step
                = wm.self().playerType()
                .cyclesToReachDistance( wm.self().pos().dist( drib_target ) );
            if ( wm.self().pos().x > 35.0 )
            {
                drib_target.y *= ( 10.0 / drib_target.absY() );
            }
            Body_Dribble( drib_target,
                          1.0,
                          ServerParam::i().maxDashPower(),
                          std::min( 5, max_dash_step )
                          ).execute( agent );
        }
        else
        {
            Body_Dribble( drib_target,
                          1.0,
                          ServerParam::i().maxDashPower(),
                          2
                          ).execute( agent );

        }
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    // opp is far from me
    if ( nearest_opp_dist > 2.5 )
    {
        Body_Dribble( drib_target,
                      1.0,
                      ServerParam::i().maxDashPower(),
                      1
                      ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    // opp is near

    if ( Body_Pass().execute( agent ) )
    {
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }


    // opp is far from me
    if ( nearest_opp_dist > 3.0 )
    {
        Body_Dribble( drib_target,
                      1.0,
                      ServerParam::i().maxDashPower(),
                      1
                      ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }


    if ( nearest_opp_dist > 2.5 )
    {
        Body_HoldBall().execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }


    {
        Body_AdvanceBall().execute( agent );
        agent->setNeckAction( new Neck_ScanField() );
    }

    return true;
}

/*bool
SamplePlayer::PassPlayersAvailable( PlayerAgent * agent ){
    const WorldModel & wm = agent->world();

    Vector2D myPosition = wm.self().pos();
    Vector2D currentHole = RoundToNearestHole(myPosition);
    Vector2D frontup = Vector2D(currentHole.x+10, currentHole.y-10);
    Vector2D backup = Vector2D(currentHole.x-10, currentHole.y-10);
    Vector2D frontdown = Vector2D(currentHole.x+10, currentHole.y+10);
    Vector2D backdown = Vector2D(currentHole.x-10, currentHole.y+10);

    Vector2D fronthor = Vector2D(currentHole.x+20, currentHole.y);
    Vector2D backhor = Vector2D(currentHole.x-20, currentHole.y);
    Vector2D upvert = Vector2D(currentHole.x, currentHole.y-20);
    Vector2D downvert = Vector2D(currentHole.x, currentHole.y+20);

    double buffer = 2.5;

    //TODO: Return true only if pass is advantageous

    if( IsOccupiedForPassing(agent, frontup, buffer)||
        IsOccupiedForPassing(agent, frontdown, buffer)||
        IsOccupiedForPassing(agent, backup, buffer)||
        IsOccupiedForPassing(agent, backdown, buffer)||
        IsOccupiedForPassing(agent, fronthor, buffer)||
        IsOccupiedForPassing(agent, upvert, buffer)||
        IsOccupiedForPassing(agent, downvert, buffer)||
        IsOccupiedForPassing(agent, backhor, buffer)
        ){
        return true;
    }

    /*
    const WorldModel & world = agent->world();
    const PlayerPtrCont::const_iterator
    t_end = world.teammatesFromSelf().end();
    for ( PlayerPtrCont::const_iterator
              it = world.teammatesFromSelf().begin();
          it != t_end;
          ++it )
    {
        if(((*it)->pos().dist(world.ball().pos()))<=20.0){
            return false;
        }
    }


    return false;
}
*/

bool
SamplePlayer::IsSectorEmpty(PlayerAgent * agent, Vector2D target, double buf_degree){
    Neck_TurnToPoint( target ).execute(agent );
    const WorldModel & wm = agent->world();
    AngleDeg target_angle = (target - wm.self().pos()).dir();
    double target_dis = wm.self().pos().dist(target);
    const Sector2D sector( wm.self().pos(),
                            0.5, target_dis + 4,
                            target_angle - buf_degree,
                            target_angle + buf_degree );
    // opponent check without goalie
    if ( ! wm.existOpponentIn( sector, 10, false ) )
    {
        return true;
    }

    return false;
}



int
SamplePlayer::ClosestPlayerToBall(PlayerAgent * agent){
    double mindis = 999;
    int mindisunum = -1;
    for(int i=2; i<=11; i++){
        if(agent->world().ourPlayer(i)!=NULL){
            if(agent->world().ourPlayer(i)->distFromBall() < mindis){
                mindis = agent->world().ourPlayer(i)->distFromBall();
                mindisunum = i;
            }
        }
    }
    return mindisunum;
}


//main function that will be used.

bool
SamplePlayer::executeSampleRole( PlayerAgent * agent )
{
    if(agent->config().teamName()=="opp"){
        Body_GoToPoint( Vector2D(-50,0), 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
        return true;
    }


    //Setting up of different flags.

    bool kickable = agent->world().self().isKickable();

    // If there is a teammate who can kick the ball and who is closer to the ball than I am.
    if ( agent->world().existKickableTeammate()
         && agent->world().teammatesFromBall().front()->distFromBall()
         < agent->world().ball().distFromSelf() )
    {
        kickable = false;
    }


    if(agent->world().existKickableTeammate()){
        //passHeard = false;
        Opponenthasball = false;
    }
    else{};
        //Opponenthasball = true;


    if(agent->world().existKickableOpponent()){
        Opponenthasball = true;
    }

    //----------XX------------//
    //If you have taken attack, you will have comment out the attach functions are replace
    //them with your own, similarly if you have taken defense, you need to comment out the existing
    //defence function and replace it with your own.
    //------------xx------------//

   //  givePass(this,passer,(passer==11)?2:passer+1);

    // I have the ball, what to do?
  //   const PlayerPtrCont & team = wm.teammatesFromSelf();
 //    const PlayerObject * nearest_team=team.front();

    //makeDribble(this,Vector2D(-10,0),8,0.5);
    setNeckAction( new Neck_TurnToBall() );
    Bhv_BodyNeckToBall().execute(this);

    if(!world().audioMemory().pass().empty() )

    {



      if(world().audioMemory().pass().front().receiver_!=world().self().unum())
      {
       #ifdef MYDEBUG
       cout<<"\rPass Count: For"<<world().self().unum()<<" is "<<world().audioMemory().pass().front().receiver_<<"  $";
       cout<<world().self().unum()<<" is "<<lastRole<<"  $"<<endl;
       #endif

        lastRole="Normal";
      }

    }

    //makeDribble(this,Vector2D(40,0),7,0.75);
    if ( kickable && !Opponenthasball)
    {
       //  doKick( this);
       //givePass(this,world().self().unum(),world().self().unum()==11?2:world().self().unum()+1);
      //   giveThrough(this,world().self().unum(),10);
     makeAttack(this);

    }

    //This is for off the ball movement which attacking, where to go for passes etc.
    else if (!kickable && !Opponenthasball)
    {


      // doMove(this);

       runThrough(this);
         if(world().audioMemory().pass().empty())
            updatePositionOf(agent,o_formation);
         else if(world().audioMemory().pass().front().receiver_!= world().self().unum())
          updatePositionOf(agent,o_formation);
    


  }

    //ATTACK ENDS HERE
    //--------XX----------XX--------//
    // DEFENCE STARTS HERE
    //The defense call
    else
    {
        //Uncomment this for defense.
        //If I can kick the ball, but opponent has it. Common ball.
        if (kickable && Opponenthasball){
            if ( Bhv_ChainAction().execute( agent ) )
            {
                dlog.addText( Logger::TEAM,
                      __FILE__": (execute) do chain action" );
            agent->debugClient().addMessage( "ChainAction" );
            return true;
            }

            Bhv_BasicOffensiveKick().execute( agent );
            return true;

            }

        // I don't have the ball, opponent has it, off the ball movement while defending.
        //falling back etc.
        else if (!kickable && Opponenthasball){
            Bhv_BasicMove().execute(agent);
        }
        return true;
    };

    //DEFENSE ENDS HERE.

    return true;
}

/*-------------------------------------------------------------------*/
/*!

*/
void
SamplePlayer::doKick( PlayerAgent * agent )
{

    if ( Bhv_ChainAction().execute( agent ) )
    {
        return;
    }

    Bhv_BasicOffensiveKick().execute( agent );
}



/*-------------------------------------------------------------------*/
/*!

*/
void
SamplePlayer::doMove( PlayerAgent * agent )
{
        Bhv_BasicMove().execute(agent);
        return;
}

bool
SamplePlayer::BasicMove(PlayerAgent * agent){
    const WorldModel & wm = agent->world();

    //-----------------------------------------------
    // tackle

    if ( Bhv_BasicTackle( 0.8, 80.0 ).execute( agent ) )
    {
        return true;
    }

    /*--------------------------------------------------------*/
    // chase ball
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();

    //Intercept
    if ( ! wm.existKickableTeammate()
         && ( self_min <= 3
              || ( self_min <= mate_min
                   && self_min < opp_min + 3 )
              )
         )
    {
        std::cout<<"body intercept called for player - "<<wm.self().unum()<<std::endl;


        dlog.addText( Logger::TEAM,
                      __FILE__": intercept");
        Body_Intercept().execute( agent );
        agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
        return true;
    }

    //Check if ball has been passed
    /*
    if(wm.existKickableTeammate()){
        int CurrentBH = GetBHUnum(agent);
        if(CurrentBH!=LastBH && CurrentBH!=-1){
            FoundNewBH = true;
            LastBH = CurrentBH;
        }
        else
            FoundNewBH = false;

        //Opponenthasball=false;
    }
    */

    const Vector2D target_point = Vector2D(0,0);
    const double dash_power = ServerParam::i().maxDashPower();

    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if ( dist_thr < 1.0 ) dist_thr = 1.0;



    if ( wm.existKickableOpponent()
         && wm.ball().distFromSelf() < 18.0 )
    {
        agent->setNeckAction( new Neck_TurnToBall() );
        return true;
    }
    else
    {
        agent->setNeckAction( new Neck_TurnToBallOrScan() );
    }

    return true;
}






//centreback







bool
SamplePlayer::isKickable(PlayerAgent * agent, int unum){
    const WorldModel & wm = agent->world();
    if(wm.ourPlayer(unum)!=NULL){
        if(wm.ourPlayer(unum)->distFromBall() < ServerParam::i().defaultKickableArea()){
            return true;
        }
        return false;
    }
    else{
        return false;
    }
}

bool
SamplePlayer::AreSamePoints(Vector2D A, Vector2D B, double buffer){
    //Check if and b are the same points +/- buffer
    if(A.dist(B)<buffer)
        return true;
    return false;
}

double
SamplePlayer::abs(double d){
    if (d>0.00)
        return d;
    else
        return d*(-1.00);
}


bool
SamplePlayer::AreSameNos(double A, double B, double buffer){
    if( abs(A-B) < buffer)
        return true;
    return false;
}



double DistanceBetweenPoints(double x1,double y1,double x2,double y2){

        double distance = sqrt(pow((x1-x2),2)+pow((y1-y2),2));

        return distance;
    }



    double AngleBetweenPoints(double x1,double y1,double x2,double y2,double x3,double y3){

        double angle = acos((pow(DistanceBetweenPoints(x1,y1,x2,y2),2)+pow(DistanceBetweenPoints(x1,y1,x3,y3),2)-pow(DistanceBetweenPoints(x2,y2,x3,y3),2))/(2*DistanceBetweenPoints(x1,y1,x2,y2)*DistanceBetweenPoints(x1,y1,x3,y3)));

        return angle;
    }

    static bool compare_first(const std::pair<double,double>& i, const std::pair<double,double>& j)
    {
        return i.first > j.first;
    }


    static bool compare_second(const std::pair<double,double>& i, const std::pair<double,double>& j)
    {
        return i.second > j.second;
    }

    double slope(double x1,double y1,double x2,double y2){

        double slope_of_line = (y2-y1)/(x2-x1);

        return slope_of_line;
    }

    double constant(double x1,double y1,double slope_of_line){

        return (y1-slope_of_line*x1);
    }

    double bisector_line_const(double c1,double c2,double a1,double a2){


        return ((c2-c1)*sqrt((pow(a2,2)+1)*(pow(a1,2)+1)));
    }

    double bisector_line_x_const(double a1,double a2){

        return ((a2*sqrt(pow(a1,2)+1))-(a1*sqrt(pow(a2,2)+1)));
    }

    double bisector_line_y_const(double a1,double a2){


        return (sqrt(pow(a2,2)+1)-sqrt(pow(a1,2)+1));
    }

    double intersecting_point_x(double c1, double c2, double a, double b){

        double x = ((c2*b - c1*a)/((a*a)+(b*b)));

        return x;
    }



    double intersecting_point_y(double c1, double c2, double a, double b){

        double y = ((c2*a + c1*b)/((a*a)+(b*b)));

        return y;
    }

    double angle_between_two_lines(const Line2D & line1,const Line2D & line2 ){

        double theta_1 = atan(line1.getB()/line1.getA());

        double theta_2 = atan(line2.getB()/line2.getA());

        return (theta_2-theta_1);
    }


    static
    Line2D angle_bisectorof( const Vector2D & origin,
                           const AngleDeg & left,
                           const AngleDeg & right )
      {
          return Line2D( origin, AngleDeg::bisect( left, right ) );
      }





/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleActionStart()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleActionEnd()
{
    if ( world().self().posValid() )
    {
#if 0
        const ServerParam & SP = ServerParam::i();
        //
        // inside of pitch
        //

        // top,lower
        debugClient().addLine( Vector2D( world().ourOffenseLineX(),
                                         -SP.pitchHalfWidth() ),
                               Vector2D( world().ourOffenseLineX(),f
                                         -SP.pitchHalfWidth() - 3.0 ),
                               Vector2D( world().ourOffensePlayerLineX(),
                                         -SP.pitchHalfWidth() ) );
        // top,upper
        debugClient().addLine( Vector2D( world().ourDefensePlayerLineX(),
                                         -SP.pitchHalfWidth() - 3.0 ),
                               Vector2D( world().ourDefensePlayerLineX(),
                                         -SP.pitchHalfWidth() ) );
        // bottom,lower
        debugClient().addLine( Vector2D( world(Opponenthasball
O).theirOffensePlayerLineX(),
                                         +SP.pitchHalfWidth() ),
                               Vector2D( world().theirOffensePlayerLineX(),
                                         +SP.pitchHalfWidth() + 3.0 ) );
        // bottom,lower
        debugClient().addLine( Vector2D( world().theirDefensePlayerLineX(),
                                         +SP.pitchHalfWidth() ),
                               Vector2D( world().theirDefensePlayerLineX(),
                                         +SP.pitchHalfWidth() + 3.0 ) );
#else
        // top,lower
        debugClient().addLine( Vector2D( world().ourDefenseLineX(),
                                         world().self().pos().y - 2.0 ),
                               Vector2D( world().ourDefenseLineX(),
                                         world().self().pos().y + 2.0 ) );

        //
        debugClient().addLine( Vector2D( world().offsideLineX(),
                                         world().self().pos().y - 15.0 ),
                               Vector2D( world().offsideLineX(),
                                         world().self().pos().y + 15.0 ) );
#endif
    }

    //
    // ball position & velocity
    //
    dlog.addText( Logger::WORLD,
                  "WM: BALL pos=(%lf, %lf), vel=(%lf, %lf, r=%lf, ang=%lf)",
                  world().ball().pos().x,
                  world().ball().pos().y,
                  world().ball().vel().x,
                  world().ball().vel().y,
                  world().ball().vel().r(),
                  world().ball().vel().th().degree() );


    dlog.addText( Logger::WORLD,
                  "WM: SELF move=(%lf, %lf, r=%lf, th=%lf)",
                  world().self().lastMove().x,
                  world().self().lastMove().y,
                  world().self().lastMove().r(),
                  world().self().lastMove().th().degree() );

    Vector2D diff = world().ball().rpos() - world().ball().rposPrev();
    dlog.addText( Logger::WORLD,
                  "WM: BALL rpos=(%lf %lf) prev_rpos=(%lf %lf) diff=(%lf %lf)",
                  world().ball().rpos().x,
                  world().ball().rpos().y,
                  world().ball().rposPrev().x,
                  world().ball().rposPrev().y,
                  diff.x,
                  diff.y );

    Vector2D ball_move = diff + world().self().lastMove();
    Vector2D diff_vel = ball_move * ServerParam::i().ballDecay();
    dlog.addText( Logger::WORLD,
                  "---> ball_move=(%lf %lf) vel=(%lf, %lf, r=%lf, th=%lf)",
                  ball_move.x,
                  ball_move.y,
                  diff_vel.x,
                  diff_vel.y,
                  diff_vel.r(),
                  diff_vel.th().degree() );
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleServerParam()
{
    if ( KickTable::instance().createTables() )
    {
        std::cerr << world().teamName() << ' '
                  << world().self().unum() << ": "
                  << " KickTable created."
                  << std::endl;
    }
    else
    {
        std::cerr << world().teamName() << ' '
                  << world().self().unum() << ": "
                  << " KickTable failed..."
                  << std::endl;
        M_client->setServerAlive( false );
    }


    if ( ServerParam::i().keepawayMode() )
    {
        std::cerr << "set Keepaway mode communication." << std::endl;
        M_communication = Communication::Ptr( new KeepawayCommunication() );
    }
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerParam()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerType()
{

}

/*-------------------------------------------------------------------*/
/*!
  communication decision.
  virtual method in super class
*/
void
SamplePlayer::communicationImpl()
{
    if ( M_communication )
    {
        M_communication->execute( this );
    }
}

/*-------------------------------------------------------------------*/
/*!
*/
bool
SamplePlayer::doPreprocess()
{
    // check tackle expires
    // check self position accuracy
    // ball search
    // check queued intention
    // check simultaneous kick

    const WorldModel & wm = this->world();

    dlog.addText( Logger::TEAM,
                  __FILE__": (doPreProcess)" );

    //
    // freezed by tackle effect
    //
        if ( wm.self().isFrozen() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": tackle wait. expires= %d",
                      wm.self().tackleExpires() );
        // face neck to ball
        this->setViewAction( new View_Tactical() );
        this->setNeckAction( new Neck_TurnToBallOrScan() );
        return true;
    }


    //
    // BeforeKickOff or AfterGoal. jump to the initial position
    //
    if ( wm.gameMode().type() == GameMode::BeforeKickOff
         || wm.gameMode().type() == GameMode::AfterGoal_ )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": before_kick_off" );
        Vector2D move_point =  Strategy::i().getPosition( wm.self().unum() );
        Bhv_CustomBeforeKickOff( move_point ).execute( this );
        this->setViewAction( new View_Tactical() );
        return true;
    }

    //
    // self localization error
    //

    if ( ! wm.self().posValid() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": invalid my pos" );
        Bhv_Emergency().execute( this ); // includes change view
        return true;
    }


    //
    // ball localization error
    //

    const int count_thr = ( wm.self().goalie()
                            ? 10
                            : 5 );
    if ( wm.ball().posCount() > count_thr
         || ( wm.gameMode().type() != GameMode::PlayOn
              && wm.ball().seenPosCount() > count_thr + 10 ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": search ball" );
        this->setViewAction( new View_Tactical() );
        Bhv_NeckBodyToBall().execute( this );
        return true;
    }


    //
    // set default change view
    //

    this->setViewAction( new View_Tactical() );

    //
    // check shoot chance
    //

    if ( doShoot() )
    {

        std::cout<<"doShoot"<<std::endl;
        std::cout<<"*******************************************************************************"<<std::endl;

        return true;
    }


    //
    // check queued action
    //

    if ( this->doIntention() )
    {
        #ifdef MYDEBUG
        std::cout<<"doIntention------------------------------------------------------------"<<std::endl;
        std::cout<<"*******************************************************************************"<<std::endl;
        dlog.addText( Logger::TEAM,
                      __FILE__": do queued intention" );
        #endif
        return true;
    }

    //
    // check simultaneous kick
    //

    if ( doForceKick() )
    {
        std::cout<<"doForceKick--------------------------------------------------------------"<<std::endl;
        std::cout<<"*******************************************************************************"<<std::endl;
        return true;
    }


    //
    // check pass message
    //

    if ( doHeardPassReceive() )
    {
        #ifdef MYDEBUG
        std::cout<<"doHeardPassReceive------------------------------------------------------"<<std::endl;
        std::cout<<"*******************************************************************************"<<std::endl;
          #endif
        return true;
    }


    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doShoot()
{
    const WorldModel & wm = this->world();

    if ( wm.gameMode().type() != GameMode::IndFreeKick_
         && wm.time().stopped() == 0
         && wm.self().isKickable()
         && Bhv_StrictCheckShoot().execute( this ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": shooted" );

        // reset intention
        this->setIntention( static_cast< SoccerIntention * >( 0 ) );
        return true;
    }

    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doForceKick()
{
    const WorldModel & wm = this->world();

    if ( wm.gameMode().type() == GameMode::PlayOn
         && ! wm.self().goalie()
         && wm.self().isKickable()
         && wm.existKickableOpponent() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": simultaneous kick" );
        this->debugClient().addMessage( "SimultaneousKick" );
        Vector2D goal_pos( ServerParam::i().pitchHalfLength(), 0.0 );

        if ( wm.self().pos().x > 36.0
             && wm.self().pos().absY() > 10.0 )
        {
            goal_pos.x = 45.0;
            dlog.addText( Logger::TEAM,
                          __FILE__": simultaneous kick cross type" );
        }
        Body_KickOneStep( goal_pos,
                          ServerParam::i().ballSpeedMax()
                          ).execute( this );
        this->setNeckAction( new Neck_ScanField() );
        return true;
    }

    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doHeardPassReceive()
{
    const WorldModel & wm = this->world();

    if ( wm.audioMemory().passTime() != wm.time()
         || wm.audioMemory().pass().empty()
         || wm.audioMemory().pass().front().receiver_ != wm.self().unum() )
    {
        return false;
    }

    //passHeard = true;

    //Vector2D closestOppPos = wm.getOpponentNearestToSelf(5, false)->pos();

    //closestOppDis = static_cast<int>(wm.self().pos().dist(closestOppPos));


    int self_min = wm.interceptTable()->selfReachCycle();
    Vector2D intercept_pos = wm.ball().inertiaPoint( self_min );
    Vector2D heard_pos = wm.audioMemory().pass().front().receive_pos_;

    dlog.addText( Logger::TEAM,
                  __FILE__":  (doHeardPassReceive) heard_pos(%.2f %.2f) intercept_pos(%.2f %.2f)",
                  heard_pos.x, heard_pos.y,
                  intercept_pos.x, intercept_pos.y );

    if ( ! wm.existKickableTeammate()
         && wm.ball().posCount() <= 1
         && wm.ball().velCount() <= 1
         && self_min < 20
         //&& intercept_pos.dist( heard_pos ) < 3.0 ) //5.0 )
         )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": (doHeardPassReceive) intercept cycle=%d. intercept",
                      self_min );
        this->debugClient().addMessage( "Comm:Receive:Intercept" );
        Body_Intercept().execute( this );
        this->setNeckAction( new Neck_TurnToBall() );
    }
    else
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": (doHeardPassReceive) intercept cycle=%d. go to receive point",
                      self_min );
        this->debugClient().setTarget( heard_pos );
        this->debugClient().addMessage( "Comm:Receive:GoTo" );
        Body_GoToPoint( heard_pos,
                    0.5,
                        ServerParam::i().maxDashPower()
                        ).execute( this );
        this->setNeckAction( new Neck_TurnToBall() );
    }

    this->setIntention( new IntentionReceive( heard_pos,
                                              ServerParam::i().maxDashPower(),
                                              0.9,
                                              5,
                                              wm.time() ) );

    return true;
}

/*-------------------------------------------------------------------*/
/*!

*/
FieldEvaluator::ConstPtr
SamplePlayer::getFieldEvaluator() const
{
    return M_field_evaluator;
}

/*-------------------------------------------------------------------*/
/*!

*/
FieldEvaluator::ConstPtr
SamplePlayer::createFieldEvaluator() const
{
    return FieldEvaluator::ConstPtr( new SampleFieldEvaluator );
}


/*-------------------------------------------------------------------*/
/*!
*/
#include "actgen_cross.h"
#include "actgen_direct_pass.h"
#include "actgen_self_pass.h"
#include "actgen_strict_check_pass.h"
#include "actgen_short_dribble.h"
#include "actgen_simple_dribble.h"
#include "actgen_shoot.h"
#include "actgen_action_chain_length_filter.h"

ActionGenerator::ConstPtr
SamplePlayer::createActionGenerator() const
{
    CompositeActionGenerator * g = new CompositeActionGenerator();

    //
    // shoot
    //
    g->addGenerator( new ActGen_RangeActionChainLengthFilter
                     ( new ActGen_Shoot(),
                       2, ActGen_RangeActionChainLengthFilter::MAX ) );

    //
    // strict check pass
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_StrictCheckPass(), 1 ) );

    //
    // cross
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_Cross(), 1 ) );

    //
    // direct pass
    //
    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
    //                  ( new ActGen_DirectPass(),
    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );

    //
    // short dribble
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_ShortDribble(), 1 ) );

    //
    // self pass (long dribble)
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_SelfPass(), 1 ) );

    //
    // simple dribble
    //
    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
    //                  ( new ActGen_SimpleDribble(),
    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );

    return ActionGenerator::ConstPtr( g );
}
