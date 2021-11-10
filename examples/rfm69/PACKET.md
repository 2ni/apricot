<table style="text-align: center;width:100%;">
<tr style="background: #dbd69d"><td colspan=14>packet <span style="font-size: .7rem; opacity: .5;">max 65bytes</span></td></tr>
<tr>
  <td style="background: #9da2dd" colspan=4>header <span style="font-size: .7rem; opacity: .5;">8bytes</span></td>
  <td style="background: #dbb69d" colspan=10>data <span style="font-size: .7rem; opacity: .5;">max 57bytes</span></td>
</tr>
<tr>
  <td style="background: #bcbfdb" >total len <span style="font-size: .7rem; opacity: .5;">1byte</span></td>
  <td style="background: #bcbfdb" >to <span style="font-size: .7rem; opacity: .5;">3bytes</span></td>
  <td style="background: #bcbfdb" >from <span style="font-size: .7rem; opacity: .5;">3bytes</span></td>
  <td style="background: #bcbfdb" >ctrl <span style="font-size: .7rem; opacity: .5;">1byte</span></td>

  <td style="background: #ddbfb1" colspan=5>datapacket <span style="font-size: .7rem; opacity: .5;">≥2bytes</span></td>
  <td style="background: #ddbfb1" colspan=4>datapacket <span style="font-size: .7rem; opacity: .5;">≥2bytes</span></td>
  <td style="background: #fff">...</td>
</tr>
<tr>
  <td style="border: 0; background: #fff;" colspan=4></td>
  <td colspan=2 style="background: #e0b760">type/len <span style="font-size: .7rem; opacity: .5;">1byte</span></td>
  <td style="background: #dbd1ce">data <span style="font-size: .7rem; opacity: .5;">1byte</span></td>
  <td style="background: #dbd1ce">data <span style="font-size: .7rem; opacity: .5;">1byte</span></td>
  <td>...</td>

  <td style="background: #e0b760">type/len <span style="font-size: .7rem; opacity: .5;">1byte</span></td>
  <td style="background: #dbd1ce">data <span style="font-size: .7rem; opacity: .5;">1byte</span></td>
  <td style="background: #dbd1ce">data <span style="font-size: .7rem; opacity: .5;">1byte</span></td>
  <td>...</td>
  <td>...</td>
</tr>
<tr style="border: 0;">
  <td style="border: 0; background: #fff;" colspan=4></td>
  <td style="background: #dbbf87">type <span style="font-size: .7rem; opacity: .5;">4bits</span>
  <td style="background: #dbbf87">len <span style="font-size: .7rem; opacity: .5;">4bits</span>
</table>
