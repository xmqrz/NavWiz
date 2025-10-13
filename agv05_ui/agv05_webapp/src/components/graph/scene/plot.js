import * as d3 from 'd3';

export default function (viz, scene, data, color) {
  const plotLine = viz
    .append('path')
    .datum(data)
    .attr('class', 'opacity-50')
    .style('stroke', color)
    .style('stroke-width', '2px')
    .style('fill', 'transparent');

  const plotArea = viz
    .append('path')
    .datum(data)
    .attr('class', 'opacity-20')
    .attr('fill', 'black');

  const points = viz.append('g');

  function updated() {
    const xScale = d3
      .scaleLinear()
      .domain([scene.graph.minX, scene.graph.maxX])
      .range([viz.meta.marginLeft, viz.meta.width - viz.meta.marginRight]);

    const yScale = d3
      .scaleLinear()
      .domain([scene.graph.minY, scene.graph.maxY])
      .range([viz.meta.height - viz.meta.marginBottom, viz.meta.marginTop]);

    const area = d3
      .area()
      .curve(d3.curveBumpX)
      .x((d) => xScale(d[0]))
      .y0(yScale(scene.graph.minY))
      .y1((d) => yScale(d[1]));

    const line = d3
      .line()
      .curve(d3.curveBumpX)
      .x((d) => xScale(d[0]))
      .y((d) => yScale(d[1]));

    plotArea.attr('d', area);
    plotLine.attr('d', line);

    const p = points.selectAll('circle').data(data).join('circle');
    p.attr('fill', color)
      .attr('r', 3)
      .merge(p)
      .attr('cx', (d) => xScale(d[0]))
      .attr('cy', (d) => yScale(d[1]));
  }
  return { updated };
}
