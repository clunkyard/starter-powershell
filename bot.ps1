# A robot on the arena's shared .NET core binding.
# The SDK launcher loads Clunkyard.dll before this file is parsed.
# Bearing is earned: a scan contact's bearing is off by up to its
# BearingError (degrees, a fixed share of the arc), so a wide beam finds a
# robot and a narrow one aims at it. This robot narrows the beam on its target
# and shoots along a line fitted through its recent sightings. Contacts within
# 120 units are exact.
# Clunkyard robot API 2.1.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude wall_contact hurt applied hull_mounts
#   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   history(slot) last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# drone: has ready in_flight launches_left report sightings
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) to_chassis(heading pitch attitude)
#   from_chassis(heading pitch attitude) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
#   drone.launch(heading pitch param) drone.brief(index value)
#   drone.program(language source) status.set(text value) debug.log(line)
#   raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
using namespace Clunkyard

class Starter : Clunkyard.Robot {
    static [double] $SearchArc=40   # degrees: sweeps the circle in half a second
    static [double] $HoldWidth=60   # units across at the target: it stays in the beam from tick to tick
    static [double] $MinArc=6       # degrees: the narrowest the beam holds
    static [int] $Window=12         # ticks of sightings the aim averages
    static [int] $Sightings=8       # narrow sightings in the window before a shot
    static [int] $LookEvery=40      # ticks between looks around the target, for a nearer robot
    static [int] $Reacquire=10      # ticks to look again where a lost target was
    [bool] $Free
    [int] $Dodges
    [int] $DodgeEnd
    [int] $Slot
    [double] Steer([double]$towards,[double]$current){return $this.Angles.Clamp($this.Angles.AngleDiff($towards,$current)/45)}
    [void] OnStart(){$this.Free=$this.Match.Is3d -and $this.Arena.Gravity.Z -eq 0}
    # A straight line through the narrowest of the last Window ticks' sightings:
    # where the robot is now, how fast it moves, and how many sightings it rests
    # on. Each sighting is off by its BearingError; the line averages that out.
    [object] Fit([int]$slot,[int]$tick){
        $recent=[System.Collections.Generic.List[object]]::new();$best=[double]::MaxValue
        foreach($s in $this.Tracker.History($slot)){
            if($s.Tick -gt $tick-[Starter]::Window){$recent.Add($s);if($s.BearingError -lt $best){$best=$s.BearingError}}
        }
        if($recent.Count -lt 2){return $null}
        $seen=[System.Collections.Generic.List[object]]::new()
        foreach($s in $recent){if($s.BearingError -le 1.5*$best){$seen.Add($s)}}
        $n=$seen.Count
        if($n -lt 2){return $null}
        $t0=0.0;foreach($s in $seen){$t0+=$s.Tick/$n}
        $spread=0.0;foreach($s in $seen){$spread+=($s.Tick-$t0)*($s.Tick-$t0)}
        $position=@(0.0,0.0,0.0);$velocity=@(0.0,0.0,0.0)
        for($axis=0;$axis -lt 3;$axis++){
            $mean=0.0;$slope=0.0
            foreach($s in $seen){$mean+=@($s.Position.X,$s.Position.Y,$s.Position.Z)[$axis]/$n}
            foreach($s in $seen){$slope+=($s.Tick-$t0)*(@($s.Position.X,$s.Position.Y,$s.Position.Z)[$axis]-$mean)}
            $slope/=$spread
            $position[$axis]=$mean+$slope*($tick-$t0);$velocity[$axis]=$slope*$this.Match.TickRate
        }
        return @{Position=[Clunkyard.Vec3]::new($position[0],$position[1],$position[2]);Velocity=[Clunkyard.Vec3]::new($velocity[0],$velocity[1],$velocity[2]);Count=$n}
    }
    [void] OnTurn(){
        $me=$this.Me;$position=$me.Position;$gun=$this.Gun;$scanner=$this.Scanner;$tick=$this.Clock.Tick
        $this.Drive.Forces($null,$null,$null,$null,$null,$null,0.2)
        $target=$this.Sensors.Nearest;$cue=$this.Tracker.Cue
        $lastSeen=if($this.Slot -ne 0){$this.Tracker.LastSeen($this.Slot)}else{$null}
        if($null -ne $target){
            $this.Slot=$target.Slot
            $throttle=if($target.Range -gt 350){0.7}elseif($target.Range -lt 200){-0.4}else{0.2}
            $this.Drive.Set($throttle,$this.Steer($target.Bearing,$me.Heading),$this.Steer($target.Elevation,$me.Pitch))
            # Halve the beam each tick until it is HoldWidth across at the target:
            # narrow enough for a precise bearing, wide enough to keep it in.
            $hold=[Math]::Max([Starter]::MinArc,2*[Math]::Atan2([Starter]::HoldWidth/2,$target.Range)*180/[Math]::PI)
            # In a crowd a narrow beam sees one robot. Every LookEvery ticks,
            # widen it for a tick: a nearer robot it finds becomes the target.
            $arc=if($tick%[Starter]::LookEvery -eq 0){[Starter]::SearchArc}else{[Math]::Max($hold,[Math]::Min([Starter]::SearchArc,$scanner.Last.Arc/2))}
            $scanner.Set($target.Bearing,$arc,$target.Elevation)
            $power=$gun.PowerForFlightTime($target.Range,1)
            # An exact contact: Gun.Lead is the intercept if the target holds
            # course. A noisy one: lead the fitted line, and fire only once it
            # rests on enough sightings.
            $ready=$true;$aim=$null
            if($target.BearingError -gt 0){
                $fit=$this.Fit($target.Slot,$tick)
                if($null -ne $fit){$aim=$gun.LeadPoint($fit.Position,$fit.Velocity,$power)}
                $ready=($null -ne $fit) -and ($fit.Count -ge [Starter]::Sightings)
            }else{$aim=$gun.Lead($target,$power)}
            $heading=if($null -ne $aim){$aim.Heading}else{$target.Bearing}
            $pitch=if($null -ne $aim){$aim.Pitch}else{$target.Elevation}
            $gun.Aim($heading,$pitch);$now=$gun.AimNow
            if($ready -and [Math]::Abs($this.Angles.AngleDiff($heading,$now.Heading)) -lt 3 -and [Math]::Abs($pitch-$now.Pitch) -lt 3 -and $gun.CanFire($power,10)){$gun.Fire($power)}
            $this.Status.Set($(if($ready){'TRACK'}else{'NARROW'}),$target.Range)
        }elseif($null -ne $lastSeen -and $tick-$lastSeen -le [Starter]::Reacquire){
            # Lost it: look again where it was last seen, widening the beam, before searching.
            $where=$this.Tracker.Position($this.Slot)
            $bearing=$this.Angles.BearingTo($position,$where);$elevation=if($this.Free){$this.Angles.ElevationTo($position,$where)}else{0}
            $this.Drive.Set(0.5,$this.Steer($bearing,$me.Heading),$this.Steer($elevation,$me.Pitch))
            $scanner.Set($bearing,[Math]::Min([Starter]::SearchArc,$scanner.Last.Arc*2),$elevation);$this.Status.Set('REACQUIRE',$null)
        }elseif($null -ne $cue){
            $bearing=$this.Angles.BearingTo($position,$cue.Position);$distance=$this.Angles.Distance($position,$cue.Position)
            $elevation=if($this.Free){$this.Angles.ElevationTo($position,$cue.Position)}else{0}
            $error=[Math]::Min($cue.BearingError,30)
            $across=($tick%5-2)/2.0*$error;$up=if($this.Free){([Math]::Floor($tick/5)%3-1)*$error/2}else{0}
            $this.Drive.Set(0.7,$this.Steer($bearing,$me.Heading),$this.Steer($elevation,$me.Pitch))
            $scanner.Set($bearing+$across,$scanner.ArcForReach($distance+100),$elevation+$up);$this.Status.Set('CUE',$distance)
        }else{
            $this.Drive.Set(0.5,0.33,$this.Steer(0,$me.Pitch))
            $pitch=if($this.Free){([Math]::Floor($tick/10)%3-1)*35}else{0}
            $scanner.Set($this.Angles.Normalize($scanner.Last.Heading+36),[Starter]::SearchArc,$pitch);$this.Status.Set('SEARCH',$null)
        }
        # A simple dodge: vary it, or predict the shooter.
        $heard=$false
        foreach($c in $this.Sensors.Cues){if($c.Source -eq [Clunkyard.Source]::Muzzle){$heard=$true}}
        if($tick -ge $this.DodgeEnd -and $heard){$this.Dodges++;$this.DodgeEnd=$tick+10}
        if($tick -lt $this.DodgeEnd){
            # Off the line of fire: left, then right next time (turn is positive left, strafe right).
            $left=$this.Dodges%2 -eq 1
            if($this.Match.Control -eq [Clunkyard.Control]::Forces){$this.Drive.Set(0.5,$null,$null);$this.Drive.Forces($null,$(if($left){-1}else{1}),$null,$null,$null,$null,$null)}
            else{$this.Drive.Set(1,$(if($left){1}else{-1}),$null)}
            $this.Status.Set($(if($left){'dodge:left'}else{'dodge:right'}),$this.Dodges)
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
