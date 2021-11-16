<style>
table.rfm69 { text-align: center; width:100%; border-collapse: separate; border-spacing: .2rem; white-space: nowrap; font-size: .875rem; }
table.rfm69 td { border: 0;  }
table.rfm69 tr.overall { background: #dbd69d }
table.rfm69 span { font-size: .625rem; opacity: .5; }
table.rfm69 tr.packet td:nth-child(1) { background: #9da2dd; }
table.rfm69 tr.packet td:nth-child(2) { background: #dbb69d; }
table.rfm69 td.header { background: #bcbfdb; }
table.rfm69 .payload { background: #ddbfb1; }
table.rfm69 td.hidden, .rfm69 tr.hidden { border: 0; background: #fff; }
table.rfm69 td.empty { background: #f7f7f7; }
table.rfm69 td.datapacket.typelen { background: #e06060; }
table.rfm69 td.datapacket.data { background: #fbe679; }
table.rfm69 td.datapacket.empty { background: linear-gradient(to right, #e06060 0%, #e06060 45%, #fff 45%, #fff 55%, #fbe679 55%); }
table.rfm69 td.data.typelen { background: #e79b9b; }
table.rfm69 td.data { background: #f9f2cb; }
table.rfm69 td.values { background: #888; color: #fff; }
table.rfm69 td.data.empty { background: linear-gradient(to right, #e79b9b 0%, #e79b9b 45%, #fff 45%, #fff 55%, #f9f2cb 55%); }
</style>

<table class="rfm69" cellpadding="10px">
<tr class="overall">
  <td colspan=15>packet <span>max 65bytes</span></td>
</tr>
<tr class="packet">
  <td colspan=4>header <span>8bytes</span></td>
  <td colspan=11>payload <span>max 57bytes</span></td>
</tr>
<tr class=>
  <td class="header">total len <span>1byte</span></td>
  <td class="header">to <span>3bytes</span></td>
  <td class="header">from <span>3bytes</span></td>
  <td class="header">ctrl <span>1byte</span></td>

  <td class="payload" colspan=5>datapacket <span>≥2bytes</span></td>
  <td class="payload" colspan=5>datapacket <span>≥2bytes</span></td>

  <td class="payload">...</td>
</tr>
<tr>
  <td class="hidden" colspan=4></td>

  <td class="datapacket typelen" colspan=2>type | len1 <span>1byte</span></td>
  <td class="datapacket data" colspan=3>data <span>len1 bytes</span></td>

  <td class="datapacket typelen" colspan=2>type | len2 <span>1byte</span></td>
  <td class="datapacket data" colspan=3>data <span>len2 bytes</span></td>
  <td class="datapacket empty">...</td>
</tr>
<tr class="hidden">
  <td class="hidden" colspan=4></td>

  <td class="data typelen">type <span>4bits</span>
  <td class="data typelen">len1 <span>4bits</span>

  <td class="data">data <span>1byte</span></td>
  <td class="data">data <span>1byte</span></td>
  <td class="data">...</td>

  <td class="data typelen">type <span>4bits</span>
  <td class="data typelen">len2 <span>4bits</span>

  <td class="data">data <span>1byte</span></td>
  <td class="data">data <span>1byte</span></td>
  <td class="data">...</td>
  <td class="data empty">...</td>
</tr>

</table>

<table class="rfm69">
  <tr class="payload"><td colspan=9>datapacket RSSI <span>1 or 2bytes</td></tr>
  <tr>
    <td colspan=2 class="datapacket typelen">type | len <span>1byte</span></td>
    <td colspan=6 class="datapacket data">data <span>1byte</span></td>
    <td class="datapacket data">data <span>1byte</span></td>
  </tr>
  <tr>
    <td class="data typelen">type <span>4bits</span></td>
    <td class="data typelen">len <span>4bits</span></td>

    <td colspan=4 class="data">ctrl <span>4bits</span></td>
    <td class="data">pwr change <span>4bits</span></td>

    <td class="data">last RSSI <span>1byte</span></td>
  </tr>
  <tr>
    <td class="values">0x03</td>
    <td class="values">1 or 2</td>

    <td class="values">limit <span>1bit</span></td>
    <td class="values">reset <span>1bit</span></td>
    <td class="values">request <span>1bit</span></td>
    <td class="values">res <span>2bits</span></td>

    <td class="values">-7 ≤ x ≤ 7</td>
    <td class="values">0 ≤ x ≤ 255</td>
  </tr>
</table>
