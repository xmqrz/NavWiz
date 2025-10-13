export default function (viz) {
  var textWidthUtil = viz.append('text');
  textWidthUtil
    .attr('class', 'text-width-util')
    .attr('font-family', 'Times,serif')
    .attr('x', -1000)
    .attr('y', -1000);

  function textWidth(text, fontSize = 14, fontWeight = 'normal', fontFamily = 'Times,serif') {
    textWidthUtil
      .attr('font-family', fontFamily)
      .attr('font-weight', fontWeight)
      .attr('font-size', fontSize);
    textWidthUtil.text(text);
    return textWidthUtil.node().getComputedTextLength();
  }

  return {
    textWidth: textWidth
  };
}
