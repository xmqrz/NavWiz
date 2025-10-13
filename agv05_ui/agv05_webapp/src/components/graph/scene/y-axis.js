import * as d3 from 'd3';

export default function (viz, scene) {
  // Add the y-axis.
  let yAxis = viz.append('g').attr('class', 'opacity-30');

  function updated() {
    if (!viz.data.plot) {
      return;
    }

    // Declare the y (vertical position) scale.
    const yScale = d3
      .scaleLinear()
      .domain([scene.graph.minY, scene.graph.maxY])
      .range([viz.meta.height - viz.meta.marginBottom, viz.meta.marginTop]);

    const axis = d3.axisLeft(yScale);

    yAxis.attr('transform', `translate(${viz.meta.marginLeft},0)`).call(axis);
  }

  return { updated };
}
