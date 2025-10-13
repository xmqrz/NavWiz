import * as d3 from 'd3';

export default function (viz, scene) {
  // Add the x-axis.
  let xAxis = viz.append('g').attr('class', 'opacity-30');

  function updated() {
    if (!viz.data.plot) {
      return;
    }

    // Declare the x (horizontal position) scale.
    const xScale = d3
      .scaleTime()
      .domain([scene.graph.minX, scene.graph.maxX])
      .range([viz.meta.marginLeft, viz.meta.width - viz.meta.marginRight]);

    const axis = d3
      .axisBottom(xScale)
      .tickFormat((d) => `${d.getHours().toString().padStart(2, '0')}:00`)
      .ticks(scene.graph.ticksX);

    xAxis
      .attr('transform', `translate(0,${viz.meta.height - viz.meta.marginBottom})`)
      .call(axis);
  }

  return { updated };
}
