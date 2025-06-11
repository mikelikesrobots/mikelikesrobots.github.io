import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';

interface PIDControllerProps {
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
  position: number;
  velocity: number;
}

const PIDController: React.FC<PIDControllerProps> = ({
  initialKp = 4.0,
  initialKi = 0.0,
  initialKd = 0.0,
  initialKff = 0.0,
  setpoint = 1.0,
  initialValue = 0,
}) => {
  const isDarkTheme = document.documentElement.getAttribute('data-theme') === 'dark';
  const [kp, setKp] = useState(initialKp);
  const [ki, setKi] = useState(initialKi);
  const [kd, setKd] = useState(initialKd);
  const [kff, setKff] = useState(initialKff);
  const [data, setData] = useState<number[]>([]);
  const [time, setTime] = useState<number[]>([]);
  const [currentTimeIndex, setCurrentTimeIndex] = useState(0);
  const [pidTerms, setPidTerms] = useState<PIDTerms[]>([]);
  const [isPlaying, setIsPlaying] = useState(false);
  const animationRef = useRef<number>();
  const svgRef = useRef<SVGSVGElement>(null);

  // Mass-spring-damper system parameters
  const mass = 1.0; // kg
  const springConstant = 0.5; // N/m
  const dampingCoefficient = 0.5; // Ns/m

  useEffect(() => {
    // Simulate mass-spring-damper system with PID control
    const simulateSystem = () => {
      const newData: number[] = [];
      const newTime: number[] = [];
      const newPidTerms: PIDTerms[] = [];
      let position = initialValue;
      let velocity = 0;
      let integral = 0;
      let lastError = 0;
      const dt = 0.01; // Time step
      const simulationTime = 20; // seconds

      for (let t = 0; t <= simulationTime; t += dt) {
        // Change setpoint at 10 seconds
        const currentSetpoint = t < 10 ? setpoint : 0;
        const error = currentSetpoint - position;
        integral += error * dt;
        const derivative = (error - lastError) / dt;
        lastError = error;

        const pTerm = kp * error;
        const iTerm = ki * integral;
        const dTerm = kd * derivative;
        const ffTerm = currentSetpoint * kff;
        const controlForce = pTerm + iTerm + dTerm + ffTerm;

        // Calculate spring and damping forces
        const springForce = springConstant * position;
        const dampingForce = dampingCoefficient * velocity;

        // Calculate acceleration using F = ma
        const acceleration = (controlForce - springForce - dampingForce) / mass;

        // Update velocity and position using Euler integration
        velocity += acceleration * dt;
        position += velocity * dt;

        newData.push(position);
        newTime.push(t);
        newPidTerms.push({
          error,
          pTerm,
          iTerm,
          dTerm,
          integral,
          derivative,
          position,
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
        setCurrentTimeIndex(prevIndex => {
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

    // Create scales
    const xScale = d3.scaleLinear()
      .domain([0, d3.max(time) || 20])
      .range([margin.left, width - margin.right]);

    // Calculate y-scale domain with padding to ensure setpoint is visible
    const dataMin = d3.min(data) || 0;
    const dataMax = d3.max(data) || 1;
    const padding = Math.max(
      Math.abs(dataMax - setpoint),
      Math.abs(dataMin - setpoint)
    ) * 0.2; // 20% padding

    const yScale = d3.scaleLinear()
      .domain([
        Math.min(dataMin, setpoint) - padding,
        Math.max(dataMax, setpoint) + padding
      ])
      .range([height - margin.bottom, margin.top]);

    // Create line generator
    const line = d3.line<number>()
      .x((_, i) => xScale(time[i]))
      .y(d => yScale(d))
      .curve(d3.curveMonotoneX);

    // Create setpoint line generator with two segments
    const setpointLine = d3.line<number>()
      .x((_, i) => xScale(time[i]))
      .y((_, i) => yScale(time[i] < 10 ? setpoint : 0))
      .curve(d3.curveMonotoneX);

    // Create area generator for integral visualization
    const area = d3.area<number>()
      .x((_, i) => xScale(time[i]))
      .y0((_, i) => yScale(time[i] < 10 ? setpoint : 0))
      .y1(d => yScale(d))
      .curve(d3.curveMonotoneX);

    // Draw the integral area (only up to current time)
    const currentData = data.slice(0, currentTimeIndex + 1);

    svg.append("path")
      .datum(currentData)
      .attr("fill", isDarkTheme ? "rgba(75, 192, 192, 0.2)" : "rgba(75, 192, 192, 0.2)")
      .attr("d", area);

    // Draw the main line
    svg.append("path")
      .datum(data)
      .attr("fill", "none")
      .attr("stroke", isDarkTheme ? "rgb(75, 192, 192)" : "rgb(75, 192, 192)")
      .attr("stroke-width", 2)
      .attr("d", line);

    // Draw the setpoint line
    svg.append("path")
      .datum(data)
      .attr("fill", "none")
      .attr("stroke", isDarkTheme ? "rgb(255, 99, 132)" : "rgb(255, 99, 132)")
      .attr("stroke-width", 2)
      .attr("stroke-dasharray", "5,5")
      .attr("d", setpointLine);

    // Draw the current time indicator
    const currentX = xScale(time[currentTimeIndex]);
    svg.append("line")
      .attr("x1", currentX)
      .attr("y1", margin.top)
      .attr("x2", currentX)
      .attr("y2", height - margin.bottom)
      .attr("stroke", isDarkTheme ? "rgb(255, 255, 255)" : "rgb(0, 0, 0)")
      .attr("stroke-width", 2);

    // Add current values text
    const currentTerms = pidTerms[currentTimeIndex] || {
      error: 0,
      pTerm: 0,
      iTerm: 0,
      dTerm: 0,
      integral: 0,
      derivative: 0,
      position: 0,
      velocity: 0,
    };

    const currentValues = [
      { label: "Error", value: currentTerms.error.toFixed(2) },
      { label: "Integral", value: currentTerms.integral.toFixed(2) },
      { label: "Derivative", value: currentTerms.derivative.toFixed(2) }
    ];

    // Determine if we should show text on the left or right of the line
    const textXOffset = currentX > width - 150 ? -10 : 10;
    const textAnchor = currentX > width - 150 ? "end" : "start";

    const textGroup = svg.append("g")
      .attr("transform", `translate(${currentX + textXOffset}, ${margin.top + 10})`);

    currentValues.forEach((item, i) => {
      textGroup.append("text")
        .attr("x", 0)
        .attr("y", i * 20)
        .attr("fill", isDarkTheme ? "#fff" : "#000")
        .attr("text-anchor", textAnchor)
        .style("font-size", "12px")
        .text(`${item.label}: ${item.value}`);
    });

    // Draw the tangent line
    if (currentTimeIndex > 0 && currentTimeIndex < data.length - 1) {
      const slope = pidTerms[currentTimeIndex].velocity;
      const dx = 0.5;
      
      const left = time[currentTimeIndex] > dx ? dx : time[currentTimeIndex];
      const right = time[currentTimeIndex] + dx < time[time.length - 1] ? dx : time[time.length - 1] - time[currentTimeIndex];

      svg.append("line")
        .attr("x1", xScale(time[currentTimeIndex] - left))
        .attr("y1", yScale(data[currentTimeIndex] - slope * left))
        .attr("x2", xScale(time[currentTimeIndex] + right))
        .attr("y2", yScale(data[currentTimeIndex] + slope * right))
        .attr("stroke", isDarkTheme ? "rgb(153, 102, 255)" : "rgb(153, 102, 255)")
        .attr("stroke-width", 2);
    }

    // Add axes
    const xAxis = d3.axisBottom(xScale);
    const yAxis = d3.axisLeft(yScale);

    svg.append("g")
      .attr("transform", `translate(0,${height - margin.bottom})`)
      .call(xAxis)
      .attr("color", isDarkTheme ? "#fff" : "#000");

    svg.append("g")
      .attr("transform", `translate(${margin.left},0)`)
      .call(yAxis)
      .attr("color", isDarkTheme ? "#fff" : "#000");

  }, [data, time, currentTimeIndex, pidTerms, isDarkTheme, setpoint]);

  const sliderStyle = {
    width: '100%',
    margin: '5px 0',
  };

  const controlPanelStyle = {
    backgroundColor: isDarkTheme ? '#2d2d2d' : '#f5f5f5',
    padding: '15px',
    borderRadius: '8px',
    marginBottom: '10px',
    color: isDarkTheme ? '#fff' : '#000',
  };

  const timeSliderContainerStyle = {
    backgroundColor: isDarkTheme ? '#2d2d2d' : '#f5f5f5',
    padding: '15px 30px',
    borderRadius: '8px',
    marginBottom: '10px',
    color: isDarkTheme ? '#fff' : '#000',
  };

  const sliderContainerStyle = {
    display: 'grid',
    gridTemplateColumns: 'repeat(2, 1fr)',
    gap: '10px',
    marginBottom: '10px',
  };

  const sliderGroupStyle = {
    marginBottom: '5px',
    minWidth: '0', // Prevents overflow
  };

  const labelStyle = {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: '5px',
    fontSize: '14px',
    whiteSpace: 'nowrap',
    gap: '8px',
  };

  const labelTextStyle = {
    flex: '1',
    minWidth: '0',
    overflow: 'hidden',
    textOverflow: 'ellipsis',
  };

  const valueStyle = {
    flex: '0 0 auto',
  };

  const handlePlayPause = () => {
    setIsPlaying(!isPlaying);
  };

  const handleReset = () => {
    setIsPlaying(false);
    setCurrentTimeIndex(0);
  };

  const handleResetPID = () => {
    setKp(initialKp);
    setKi(initialKi);
    setKd(initialKd);
    setKff(initialKff);
  };

  const buttonStyle = {
    padding: '8px 16px',
    margin: '0 8px',
    borderRadius: '4px',
    border: 'none',
    backgroundColor: isDarkTheme ? '#4d4d4d' : '#e5e5e5',
    color: isDarkTheme ? '#fff' : '#000',
    cursor: 'pointer',
    display: 'inline-flex',
    alignItems: 'center',
    gap: '8px',
  };

  const buttonContainerStyle = {
    display: 'flex',
    justifyContent: 'center',
    marginTop: '10px',
  };

  return (
    <div style={{ width: '100%', maxWidth: '800px', margin: '0 auto' }}>
      <div style={controlPanelStyle}>
        <div style={sliderContainerStyle}>
          <div style={sliderGroupStyle}>
            <div style={labelStyle}>
              <span style={labelTextStyle}>Proportional Gain (Kp):</span>
              <span style={valueStyle}>{kp.toFixed(2)}</span>
            </div>
            <input
              type="range"
              id="kp"
              min="0"
              max="20"
              step="0.1"
              value={kp}
              onChange={(e) => setKp(parseFloat(e.target.value))}
              style={sliderStyle}
            />
          </div>
          <div style={sliderGroupStyle}>
            <div style={labelStyle}>
              <span style={labelTextStyle}>Integral Gain (Ki):</span>
              <span style={valueStyle}>{ki.toFixed(2)}</span>
            </div>
            <input
              type="range"
              id="ki"
              min="0"
              max="20"
              step="0.1"
              value={ki}
              onChange={(e) => setKi(parseFloat(e.target.value))}
              style={sliderStyle}
            />
          </div>
          <div style={sliderGroupStyle}>
            <div style={labelStyle}>
              <span style={labelTextStyle}>Derivative Gain (Kd):</span>
              <span style={valueStyle}>{kd.toFixed(2)}</span>
            </div>
            <input
              type="range"
              id="kd"
              min="0"
              max="20"
              step="0.1"
              value={kd}
              onChange={(e) => setKd(parseFloat(e.target.value))}
              style={sliderStyle}
            />
          </div>
          <div style={sliderGroupStyle}>
            <div style={labelStyle}>
              <span style={labelTextStyle}>Feed Forward Gain (Kff):</span>
              <span style={valueStyle}>{kff.toFixed(2)}</span>
            </div>
            <input
              type="range"
              id="kff"
              min="0"
              max="10"
              step="0.1"
              value={kff}
              onChange={(e) => setKff(parseFloat(e.target.value))}
              style={sliderStyle}
            />
          </div>
        </div>
        <div style={buttonContainerStyle}>
          <button onClick={handlePlayPause} style={buttonStyle}>
            {isPlaying ? '⏸ Pause' : '▶ Play'}
          </button>
          <button onClick={handleReset} style={buttonStyle}>
            ⏮ Reset
          </button>
          <button onClick={handleResetPID} style={buttonStyle}>
            ↺ Reset PID
          </button>
        </div>
      </div>
      <svg
        ref={svgRef}
        width="100%"
        height="400"
        style={{
          backgroundColor: isDarkTheme ? '#1a1a1a' : '#ffffff',
          borderRadius: '8px',
          marginBottom: '10px',
        }}
      />
      <div style={timeSliderContainerStyle}>
        <div style={labelStyle}>
          <label htmlFor="time">Time:</label>
          <span>{time[currentTimeIndex]?.toFixed(1)}s</span>
        </div>
        <input
          type="range"
          id="time"
          min="0"
          max={time.length - 1}
          step="1"
          value={currentTimeIndex}
          onChange={(e) => setCurrentTimeIndex(parseInt(e.target.value))}
          style={sliderStyle}
        />
      </div>
    </div>
  );
};

export default PIDController;
