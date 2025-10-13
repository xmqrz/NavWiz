const DISABLEDTEXTSTYLE =
  '-webkit-touch-callout: none;-webkit-user-select: none;-khtml-user-select: none;-moz-user-select: none;-ms-user-select: none;user-select: none;';

export default function (viz, scene) {
  let legend = viz.append('g').attr('class', 'legend');

  function updated() {
    if (!viz.data.legend) {
      return;
    }

    let textColor = getComputedStyle(viz.node()).color;

    let labelPadding = 5;
    let curX = 0;
    let data = viz.data.legend.map((label, i) => {
      let color = viz.meta.colorList[i % viz.meta.colorList.length];

      let colorX = curX + 9;
      curX += 18;
      let labelWidth = scene.utils.textWidth(label, 12, '', '');
      let labelX = curX + 0.1;
      curX += labelWidth + 0.2;
      curX += labelPadding;
      return {
        label: label,
        labelX: labelX,
        color: color,
        colorX: colorX
      };
    });

    let startX = viz.meta.width - viz.meta.marginRight - curX + labelPadding;

    let l = legend
      .selectAll('g')
      .data(data)
      .join((enter) => {
        let lg = enter.append('g');
        lg.append('circle').attr('r', 6).attr('cy', 15.5);
        lg.append('text').attr('style', DISABLEDTEXTSTYLE).attr('font-size', 12).attr('y', 20);
        return lg;
      });
    l.selectAll('circle')
      .attr('fill', (d) => d.color)
      .attr('cx', (d) => startX + d.colorX);
    l.selectAll('text')
      .attr('x', (d) => startX + d.labelX)
      .attr('fill', textColor)
      .text((d) => d.label);
  }

  return { updated };
}
