import React, { useEffect, useRef, useState } from "react";
import * as d3 from "d3";
import "./PIDController.css";

export enum PIDControllerFields {
  Kp,
  Ki,
  Kd,
  Kff,
}

interface PIDControllerProps {
  fields?: PIDControllerFields[];
  initialKp?: number;
  initialKi?: number;
  initialKd?: number;
  initialKff?: number;
  setpoint?: number;
  initialValue?: number;
}

interface PIDTerms {
  error: number;
  pTerm: number;
  iTerm: number;
  dTerm: number;
  integral: number;
  derivative: number;
  velocity: number;
}

const clamp = (value: number, min: number, max: number): number => {
  return Math.max(min, Math.min(max, value));
}

const PIDController: React.FC<PIDControllerProps> = ({
  fields = [
    PIDControllerFields.Kp,
    PIDControllerFields.Ki,
    PIDControllerFields.Kd,
    PIDControllerFields.Kff,
  ],
  initialKp = 4.0,
  initialKi = 0.0,
  initialKd = 0.0,
  initialKff = 0.0,
  setpoint = 1.0,
  initialValue = 0,
}) => {
  const hasKp = fields.includes(PIDControllerFields.Kp);
  const hasKi = fields.includes(PIDControllerFields.Ki);
  const hasKd = fields.includes(PIDControllerFields.Kd);
  const hasKff = fields.includes(PIDControllerFields.Kff);

  const [kp, setKp] = useState(hasKp ? initialKp : 0.0);
  const [ki, setKi] = useState(hasKi ? initialKi : 0.0);
  const [kd, setKd] = useState(hasKd ? initialKd : 0.0);
  const [kff, setKff] = useState(hasKff ? initialKff : 0.0);

  const [data, setData] = useState<number[]>([]);
  const [time, setTime] = useState<number[]>([]);
  const [currentTimeIndex, setCurrentTimeIndex] = useState(0);
  const [pidTerms, setPidTerms] = useState<PIDTerms[]>([]);
  const [isPlaying, setIsPlaying] = useState(false);
  const animationRef = useRef<number>();
  const svgRef = useRef<SVGSVGElement>(null);
  const [themeChangeTrigger, setThemeChangeTrigger] = useState(0);

  // Watch for theme changes
  useEffect(() => {
    const observer = new MutationObserver((mutations) => {
      mutations.forEach((mutation) => {
        if (mutation.attributeName === "data-theme") {
          setThemeChangeTrigger((prev) => prev + 1);
        }
      });
    });

    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ["data-theme"],
    });

    return () => observer.disconnect();
  }, []);

  // Robot arm joint system parameters
  const maxForce = 10.0; // N
  const massOfArm = 10.0; // kg
  const dampingCoefficient = 0.5;

  useEffect(() => {
    // Simulate mass-spring-damper system with PID control
    const simulateSystem = () => {
      const newData: number[] = [];
      const newTime: number[] = [];
      const newPidTerms: PIDTerms[] = [];
      let velocity = initialValue;
      let acceleration = 0;
      let integral = 0;
      let lastError = 0;
      const dt = 0.01; // Time step
      const simulationTime = 20; // seconds

      for (let t = 0; t <= simulationTime; t += dt) {

        // Calculate PID control terms
        const currentSetpoint = t < 10 ? setpoint : 0;
        const error = currentSetpoint - velocity;
        integral += error * dt;
        const derivative = (error - lastError) / dt;
        lastError = error;
        const pTerm = kp * error;
        const iTerm = ki * integral;
        const dTerm = kd * derivative;
        const ffTerm = velocity * kff;
        const dampingForce = -dampingCoefficient * velocity;

        const appliedForce = clamp(pTerm + iTerm + dTerm + ffTerm + dampingForce, -maxForce, maxForce);
        acceleration += appliedForce / massOfArm;
        velocity += acceleration * dt;

        newData.push(velocity);
        newTime.push(t);
        newPidTerms.push({
          error,
          pTerm,
          iTerm,
          dTerm,
          integral,
          derivative,
          velocity,
        });
      }

      setData(newData);
      setTime(newTime);
      setPidTerms(newPidTerms);
    };

    simulateSystem();
  }, [kp, ki, kd, kff, setpoint, initialValue]);

  // Animation effect
  useEffect(() => {
    if (isPlaying) {
      const animate = () => {
        setCurrentTimeIndex((prevIndex) => {
          if (prevIndex >= time.length - 1) {
            setIsPlaying(false);
            return prevIndex;
          }
          // Skip every other frame to make it 2x faster
          return prevIndex + 2;
        });
        animationRef.current = requestAnimationFrame(animate);
      };
      animationRef.current = requestAnimationFrame(animate);
    }

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isPlaying, time.length]);

  useEffect(() => {
    if (!svgRef.current || data.length === 0) return;

    const svg = d3.select(svgRef.current);
    const width = svgRef.current.clientWidth;
    const height = 400;
    const margin = { top: 20, right: 30, bottom: 30, left: 40 };

    // Clear previous content
    svg.selectAll("*").remove();

    // Get current theme
    const isDarkTheme =
      document.documentElement.getAttribute("data-theme") === "dark";
    const textColor = isDarkTheme ? "#fff" : "#000";
    const lineColor = isDarkTheme ? "#fff" : "#000";

    // Create scales
    const xScale = d3
      .scaleLinear()
      .domain([0, d3.max(time) || 20])
      .range([margin.left, width - margin.right]);

    // Calculate y-scale domain with padding to ensure setpoint is visible
    const dataMin = d3.min(data) || 0;
    const dataMax = d3.max(data) || 1;
    const padding =
      Math.max(Math.abs(dataMax - setpoint), Math.abs(dataMin - setpoint)) *
      0.2; // 20% padding

    const yScale = d3
      .scaleLinear()
      .domain([
        Math.min(dataMin, setpoint) - padding,
        Math.max(dataMax, setpoint) + padding,
      ])
      .range([height - margin.bottom, margin.top]);

    // Create line generator
    const line = d3
      .line<number>()
      .x((_, i) => xScale(time[i]))
      .y((d) => yScale(d))
      .curve(d3.curveMonotoneX);

    // Create setpoint line generator with two segments
    const setpointLine = d3
      .line<number>()
      .x((_, i) => xScale(time[i]))
      .y((_, i) => yScale(time[i] < 10 ? setpoint : 0))
      .curve(d3.curveMonotoneX);

    // Create area generator for integral visualization
    const area = d3
      .area<number>()
      .x((_, i) => xScale(time[i]))
      .y0((_, i) => yScale(time[i] < 10 ? setpoint : 0))
      .y1((d) => yScale(d))
      .curve(d3.curveMonotoneX);

    // Draw the integral area (only up to current time)
    const currentData = data.slice(0, currentTimeIndex + 1);

    svg
      .append("path")
      .datum(currentData)
      .attr("fill", "rgba(75, 192, 192, 0.2)")
      .attr("d", area);

    // Draw the main line
    svg
      .append("path")
      .datum(data)
      .attr("fill", "none")
      .attr("stroke", "rgb(75, 192, 192)")
      .attr("stroke-width", 2)
      .attr("d", line);

    // Draw the setpoint line
    svg
      .append("path")
      .datum(data)
      .attr("fill", "none")
      .attr("stroke", "rgb(255, 99, 132)")
      .attr("stroke-width", 2)
      .attr("stroke-dasharray", "5,5")
      .attr("d", setpointLine);

    // Draw the current time indicator
    const currentX = xScale(time[currentTimeIndex]);
    svg
      .append("line")
      .attr("x1", currentX)
      .attr("y1", margin.top)
      .attr("x2", currentX)
      .attr("y2", height - margin.bottom)
      .attr("stroke", lineColor)
      .attr("stroke-width", 2);

    // Add current values text
    const currentTerms = pidTerms[currentTimeIndex] || {
      error: 0,
      pTerm: 0,
      iTerm: 0,
      dTerm: 0,
      integral: 0,
      derivative: 0,
      velocity: 0,
    };

    const currentValues = [
      { label: "Error", value: currentTerms.error.toFixed(2) },
      { label: "Integral", value: currentTerms.integral.toFixed(2) },
      { label: "Derivative", value: currentTerms.derivative.toFixed(2) },
    ];

    // Determine if we should show text on the left or right of the line
    const textXOffset = currentX > width - 150 ? -10 : 10;
    const textAnchor = currentX > width - 150 ? "end" : "start";

    const textGroup = svg
      .append("g")
      .attr(
        "transform",
        `translate(${currentX + textXOffset}, ${margin.top + 10})`
      );

    currentValues.forEach((item, i) => {
      textGroup
        .append("text")
        .attr("x", 0)
        .attr("y", i * 20)
        .attr("fill", textColor)
        .attr("text-anchor", textAnchor)
        .style("font-size", "12px")
        .text(`${item.label}: ${item.value}`);
    });

    // Draw the tangent line
    if (currentTimeIndex > 0 && currentTimeIndex < data.length - 1) {
      const slope = -pidTerms[currentTimeIndex].derivative;
      const dx = 0.5;

      const left = time[currentTimeIndex] > dx ? dx : time[currentTimeIndex];
      const right =
        time[currentTimeIndex] + dx < time[time.length - 1]
          ? dx
          : time[time.length - 1] - time[currentTimeIndex];

      svg
        .append("line")
        .attr("x1", xScale(time[currentTimeIndex] - left))
        .attr("y1", yScale(data[currentTimeIndex] - slope * left))
        .attr("x2", xScale(time[currentTimeIndex] + right))
        .attr("y2", yScale(data[currentTimeIndex] + slope * right))
        .attr("stroke", "rgb(153, 102, 255)")
        .attr("stroke-width", 2);
    }

    // Add axes
    const xAxis = d3.axisBottom(xScale);
    const yAxis = d3.axisLeft(yScale);

    svg
      .append("g")
      .attr("transform", `translate(0,${height - margin.bottom})`)
      .call(xAxis)
      .attr("color", textColor)
      .style("font-size", "12px");

    svg
      .append("g")
      .attr("transform", `translate(${margin.left},0)`)
      .call(yAxis)
      .attr("color", textColor)
      .style("font-size", "12px");

    // Add grid lines
    svg
      .append("g")
      .attr("transform", `translate(0,${height - margin.bottom})`)
      .call(
        d3
          .axisBottom(xScale)
          .tickSize(-height + margin.top + margin.bottom)
          .tickFormat(() => "")
      )
      .attr(
        "color",
        isDarkTheme ? "rgba(255, 255, 255, 0.1)" : "rgba(0, 0, 0, 0.1)"
      );

    svg
      .append("g")
      .attr("transform", `translate(${margin.left},0)`)
      .call(
        d3
          .axisLeft(yScale)
          .tickSize(-width + margin.left + margin.right)
          .tickFormat(() => "")
      )
      .attr(
        "color",
        isDarkTheme ? "rgba(255, 255, 255, 0.1)" : "rgba(0, 0, 0, 0.1)"
      );
  }, [data, time, currentTimeIndex, pidTerms, setpoint, themeChangeTrigger]);

  const handlePlayPause = () => {
    setIsPlaying(!isPlaying);
  };

  const handleReset = () => {
    setIsPlaying(false);
    setCurrentTimeIndex(0);
  };

  const handleResetPID = () => {
    setKp(hasKp ? initialKp : 0.0);
    setKi(hasKi ? initialKi : 0.0);
    setKd(hasKd ? initialKd : 0.0);
    setKff(hasKff ? initialKff : 0.0);
  };

  const kpSlider = hasKp ? (
    <div className="slider-group">
      <div className="label">
        <span className="label-text">Proportional Gain (Kp):</span>
        <span className="value">{kp.toFixed(2)}</span>
      </div>
      <input
        type="range"
        id="kp"
        min="0"
        max="5"
        step="0.1"
        value={kp}
        onChange={(e) => setKp(parseFloat(e.target.value))}
        className="slider"
      />
    </div>
  ) : null;

  const kiSlider = hasKi ? (
    <div className="slider-group">
      <div className="label">
        <span className="label-text">Integral Gain (Ki):</span>
        <span className="value">{ki.toFixed(2)}</span>
      </div>
      <input
        type="range"
        id="ki"
        min="0"
        max="5"
        step="0.1"
        value={ki}
        onChange={(e) => setKi(parseFloat(e.target.value))}
        className="slider"
      />
    </div>
  ) : null;

  const kdSlider = hasKd ? (
    <div className="slider-group">
      <div className="label">
        <span className="label-text">Derivative Gain (Kd):</span>
        <span className="value">{kd.toFixed(2)}</span>
      </div>
      <input
        type="range"
        id="kd"
        min="0"
        max="5"
        step="0.1"
        value={kd}
        onChange={(e) => setKd(parseFloat(e.target.value))}
        className="slider"
      />
    </div>
  ) : null;

  const kffSlider = hasKff ? (
    <div className="slider-group">
      <div className="label">
        <span className="label-text">Feed Forward Gain (Kff):</span>
        <span className="value">{kff.toFixed(2)}</span>
      </div>
      <input
        type="range"
        id="kff"
        min="0"
        max="5"
        step="0.01"
        value={kff}
        onChange={(e) => setKff(parseFloat(e.target.value))}
        className="slider"
      />
    </div>
  ) : null;

  return (
    <div className="pid-container">
      <div className="control-panel">
        <div className="slider-container">
          {kpSlider}
          {kiSlider}
          {kdSlider}
          {kffSlider}
        </div>
        <div className="button-container">
          <button onClick={handlePlayPause} className="button">
            {isPlaying ? "⏸ Pause" : "▶ Play"}
          </button>
          <button onClick={handleReset} className="button">
            ⏮ Reset
          </button>
          <button onClick={handleResetPID} className="button">
            ↺ Reset PID
          </button>
        </div>
      </div>
      <svg ref={svgRef} className="graph" />
      <div className="time-slider-container">
        <div className="label">
          <span className="label-text">Time:</span>
          <span className="value">{time[currentTimeIndex]?.toFixed(1)}s</span>
        </div>
        <input
          type="range"
          id="time"
          min="0"
          max={time.length - 1}
          step="1"
          value={currentTimeIndex}
          onChange={(e) => setCurrentTimeIndex(parseInt(e.target.value))}
          className="slider"
        />
      </div>
    </div>
  );
};

export default PIDController;
