import React, { useEffect, useRef } from 'react';
import * as echarts from 'echarts';

/**
 * RealtimeChart — High-performance ECharts wrapper
 * Uses raw echarts instance (not echarts-for-react) for max performance at 10Hz.
 *
 * Props:
 *   times       {number[]}  - Unix timestamps (seconds) for x-axis
 *   series      {Array}     - [{name, data[], color, type?, yAxisIndex?}]
 *   windowSizeS {number}    - Rolling window in seconds (default 30)
 *   paused      {boolean}   - Freeze x-axis scrolling
 *   height      {number}    - Chart pixel height
 *   yAxis       {Array}     - ECharts yAxis config array
 */
export default function RealtimeChart({
  times = [],
  series = [],
  windowSizeS = 30,
  paused = false,
  height = 220,
  yAxis = [{ type: 'value' }],
}) {
  const containerRef   = useRef(null);
  const chartRef       = useRef(null);
  const pauseRef       = useRef(paused);
  const windowRef      = useRef(windowSizeS);
  const prevTimesLen   = useRef(0);

  // Sync refs so stable callbacks see latest props
  useEffect(() => { pauseRef.current = paused; },     [paused]);
  useEffect(() => { windowRef.current = windowSizeS; }, [windowSizeS]);

  // --- Init chart once ---
  useEffect(() => {
    if (!containerRef.current) return;

    const chart = echarts.init(containerRef.current, null, { renderer: 'canvas' });
    chartRef.current = chart;

    const BASE_OPTION = {
      backgroundColor: 'transparent',
      animation: false,
      tooltip: {
        trigger: 'axis',
        axisPointer: { type: 'cross', lineStyle: { color: 'rgba(255,255,255,0.3)' } },
        backgroundColor: 'rgba(10,14,23,0.95)',
        borderColor:     'rgba(255,255,255,0.1)',
        textStyle:       { color: '#f1f5f9', fontSize: 12, fontFamily: 'JetBrains Mono, monospace' },
        formatter: params => params.map(p =>
          `<span style="display:inline-block;width:8px;height:8px;border-radius:50%;background:${p.color};margin-right:4px;"></span>` +
          `${p.seriesName}: <b>${typeof p.value[1] === 'number' ? p.value[1].toFixed(4) : p.value[1]}</b>`
        ).join('<br/>'),
      },
      legend: {
        bottom: 0,
        left: 'center',
        icon: 'circle',
        itemWidth: 8, itemHeight: 8,
        textStyle: { color: '#94a3b8', fontSize: 11 },
      },
      grid: { left: 54, right: 18, top: 12, bottom: 36, containLabel: false },
      xAxis: {
        type: 'value',
        min: 'dataMin',
        max: 'dataMax',
        axisLine:  { lineStyle: { color: 'rgba(255,255,255,0.1)' } },
        splitLine: { lineStyle: { color: 'rgba(255,255,255,0.04)' } },
        axisLabel: {
          color: '#64748b',
          fontSize: 10,
          fontFamily: 'JetBrains Mono, monospace',
          formatter: val => {
            const d = new Date(val * 1000);
            return `${d.getHours().toString().padStart(2,'0')}:${d.getMinutes().toString().padStart(2,'0')}:${d.getSeconds().toString().padStart(2,'0')}`;
          },
        },
      },
      yAxis: yAxis.map(y => ({
        type: 'value',
        ...y,
        splitLine: { lineStyle: { color: 'rgba(255,255,255,0.05)' } },
        axisLine:  { show: false },
        axisTick:  { show: false },
        axisLabel: { color: '#64748b', fontSize: 10, fontFamily: 'JetBrains Mono, monospace' },
      })),
      series: series.map(s => ({
        name: s.name,
        type: s.type || 'line',
        showSymbol: false,
        smooth: false,
        lineStyle:  { width: 1.5, color: s.color },
        itemStyle:  { color: s.color },
        areaStyle: s.area ? { color: s.color, opacity: 0.1 } : undefined,
        yAxisIndex: s.yAxisIndex || 0,
        animation:  false,
        data: [],
        emphasis: { disabled: true },
        large: true,
        largeThreshold: 500,
      })),
    };

    chart.setOption(BASE_OPTION, true);

    const handleResize = () => chart.resize();
    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
      chart.dispose();
    };
  }, []); // Init once

  // --- Update data ---
  useEffect(() => {
    const chart = chartRef.current;
    if (!chart || times.length === 0) return;

    const latestTime = times[times.length - 1];
    const windowMin  = latestTime - windowRef.current;

    // Build paired [time, value] data for each series
    const seriesUpdate = series.map(s => {
      const data = (s.data || []).map((v, i) => [times[i], v]);
      return { data };
    });

    const update = { series: seriesUpdate };

    if (!pauseRef.current) {
      update.xAxis = { min: windowMin, max: latestTime };
    }

    chart.setOption(update, { lazyUpdate: true });
  }, [times, series]);

  return (
    <div
      ref={containerRef}
      style={{ width: '100%', height: height }}
    />
  );
}
