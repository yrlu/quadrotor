classdef FlightCorridor < handle
    %FLIGHTCORRIDOR: a light corridor defined in terms of a rotated cuboid.
    
    properties (Access = public)
        sfc = [1,1,1]'; % The safe corridor distance to three dimensions
        a = [0,0,0]'; % The start point
        b = [0,0,0]'; % The end point
        l = 0; % Corridor length
        yaw = 0;
        pitch = 0;
        G2L; % Transformation matrix global to local
        L2G; % Transformation matrix local to global
        vMc; % Max velocity in local frame
        aMc; % Max acceleration in local frame
        jMc; % Max jerk in local frame
        target; %Target in local frame
    end
    
    methods
        function this = FlightCorridor(a,b,sfc,vM,aM,jM)
            this.a = a;
            this.b = b;
            this.sfc = sfc;
            this.l = norm(b-a);
            [this.G2L,this.L2G,this.yaw,this.pitch] = global2local(a,b);
            [this.vMc,this.aMc,this.jMc]=tiltConstraintsApprx(vM,aM,jM,this.pitch);
            this.target = [this.l 0 0]';
        end
        %---
        function sfc = getSfc(this,i)
            sfc = this.sfc(i);
        end
        %---
        function yaw = getYaw(this)
            yaw = this.yaw;
        end
        %---
        function l = getL(this)
            l = this.l;
        end
        %---
        function pitch = getPitch(this)
            pitch = this.pitch;
        end
        %---
        function out = getVmc(this,i)
            out = this.vMc(i);
        end
        %---
        function out = getAmc(this,i)
            out = this.aMc(i);
        end
        %---
        function out = getJmc(this,i)
            out = this.jMc(i);
        end
        %---
        function target = getTarget(this,i)
            target = this.target(i);
        end
        %---
        function vec_out = l2g(this, vec_in,do_translation)
            if(do_translation)
                vec_out = transRot(this.L2G.pos, vec_in);
            else
                vec_out = transRot(this.L2G.free,vec_in);
            end
        end
        %---
        function vec_out = g2l(this, vec_in,do_translation)
            if(do_translation)
                vec_out = transRot(this.G2L.pos, vec_in);
            else
                vec_out = transRot(this.G2L.free,vec_in);
            end
        end
        %---
        function G2L = getG2L(this)
            G2L = this.G2L;
        end
        %---
        function L2G = getL2G(this)
            L2G = this.L2G;
        end
        %---
        function inside = isPointInside(this,p,is_local_cord)
            if (~is_local_cord)
                p = this.g2l(p);
            end
            
            inside = this.isInsideLocalCuboid(p);
        end
        %---
        function inside = isInsideLocalCuboid(this,p)
            inside = true;
            %---Consider the all three face
            if (p(1) < -this.sfc(1) || p(1) > this.sfc(1)+this.l)
                inside = false;
                return;
            end
            for i=2:3
                if(abs(p(i))>this.sfc(i))
                    inside = false;
                    return;
                end
            end
        end
        %---
        function inside = isMaxMinValueInside(this,max_p,min_p)
            inside = true;
            
            if max_p(1) < -this.sfc(1) || max_p(1) > this.sfc(1) + this.l
                inside = false;
                return;
            end
            if min_p(1) < -this.sfc(1) || min_p(1) > this.sfc(1) + this.l
                inside = false;
                return;
            end
            
            for i=2:3
                if abs(max_p(i))>this.sfc(i) || abs(min_p(i))>this.sfc(i) 
                    inside = false;
                    return;
                end
            end
        end
    end
end

