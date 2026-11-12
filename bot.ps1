# A robot on the arena's shared .NET core binding.
# The SDK launcher loads Clunkyard.dll before this file is parsed.
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range)
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
using namespace Clunkyard

class Starter : Clunkyard.Robot {
    [bool] $Free
    [double] Steer([double]$towards,[double]$current){return $this.Angles.Clamp($this.Angles.AngleDiff($towards,$current)/45)}
    [void] OnStart(){$this.Free=$this.Match.Is3d -and $this.Arena.Gravity.Z -eq 0}
    [void] OnTurn(){
        $me=$this.Me;$position=$me.Position;$gun=$this.Gun;$scanner=$this.Scanner
        $this.Drive.Forces($null,$null,$null,$null,$null,$null,0.2)
        $target=$this.Sensors.Nearest;$cue=$this.Tracker.Cue
        if($null -ne $target){
            $throttle=if($target.Range -gt 350){0.7}elseif($target.Range -lt 200){-0.4}else{0.2}
            $this.Drive.Set($throttle,$this.Steer($target.Bearing,$me.Heading),$this.Steer($target.Elevation,$me.Pitch))
            $scanner.Set($target.Bearing,20,$target.Elevation)
            $power=$gun.PowerForFlightTime($target.Range,1);$aim=$gun.Lead($target,$power)
            $heading=if($null -ne $aim){$aim.Heading}else{$target.Bearing}
            $pitch=if($null -ne $aim){$aim.Pitch}else{$target.Elevation}
            $gun.Aim($heading,$pitch);$now=$gun.AimNow
            if([Math]::Abs($this.Angles.AngleDiff($heading,$now.Heading)) -lt 3 -and [Math]::Abs($pitch-$now.Pitch) -lt 3 -and $gun.CanFire($power,10)){$gun.Fire($power)}
            $this.Status.Set('TRACK',$target.Range)
        }elseif($null -ne $cue){
            $bearing=$this.Angles.BearingTo($position,$cue.Position);$distance=$this.Angles.Distance($position,$cue.Position)
            $elevation=if($this.Free){$this.Angles.ElevationTo($position,$cue.Position)}else{0}
            $error=[Math]::Min($cue.BearingError,30);$tick=$this.Clock.Tick
            $across=($tick%5-2)/2.0*$error;$up=if($this.Free){([Math]::Floor($tick/5)%3-1)*$error/2}else{0}
            $this.Drive.Set(0.7,$this.Steer($bearing,$me.Heading),$this.Steer($elevation,$me.Pitch))
            $scanner.Set($bearing+$across,$scanner.ArcForReach($distance+100),$elevation+$up);$this.Status.Set('CUE',$distance)
        }else{
            $this.Drive.Set(0.5,0.33,$this.Steer(0,$me.Pitch))
            $pitch=if($this.Free){([Math]::Floor($this.Clock.Tick/10)%3-1)*35}else{0}
            $scanner.Set($this.Angles.Normalize($scanner.Last.Heading+36),40,$pitch);$this.Status.Set('SEARCH',$null)
        }
        if($this.Battery.Charge -lt $this.Battery.Capacity/4){$this.Drive.Set($this.Drive.Get('throttle')*0.5,$null,$null)}
        $storm=$this.Nav.Storm;$zone=$this.Nav.Zone
        if($null -ne $storm -and $storm.Margin($position) -gt -60){
            $away=$this.Angles.BearingTo($storm.Center,$position)
            $this.Drive.Set(0.8,$this.Angles.Clamp($this.Angles.AngleDiff($away,$me.Heading)/30),$null)
        }
        if($null -ne $zone -and $zone.Margin($position) -lt 60){
            $climb=if($zone.Shape -eq [Clunkyard.Shape]::Sphere){$this.Angles.ElevationTo($position,$zone.Center)}else{0}
            $this.Drive.Set(0.8,$this.Angles.Clamp($this.Angles.AngleDiff($this.Angles.BearingTo($position,$zone.Center),$me.Heading)/30),$this.Angles.Clamp(($climb-$me.Pitch)/30))
        }
    }
}

[Clunkyard.Clunkyard]::Run([Starter]::new())
