<style>
#rfm69 { text-align: center; width:100%; border-collapse: separate; border-spacing: .2rem; white-space: nowrap; font-size: .875rem; }
#rfm69 td { border: 0;  }
#rfm69 tr.overall { background: #dbd69d }
#rfm69 span { font-size: .625rem; opacity: .5; }
#rfm69 tr.packet td:nth-child(1) { background: #9da2dd; }
#rfm69 tr.packet td:nth-child(2) { background: #dbb69d; }
#rfm69 td.header { background: #bcbfdb; }
#rfm69 td.payload { background: #ddbfb1; }
#rfm69 td.hidden, #rfm69 tr.hidden { border: 0; background: #fff; }
#rfm69 td.empty { background: #f7f7f7; }
#rfm69 td.datapacket:nth-child(2), #rfm69 td.datapacket:nth-child(4) { background: #e06060; }
#rfm69 td.datapacket { background: #fbe679; }
#rfm69 td.datapacket.empty { background: linear-gradient(to right, #e06060 0%, #e06060 45%, #fff 45%, #fff 55%, #fbe679 55%); }
#rfm69 td.typelen { background: #e79b9b; }
#rfm69 td.data { background: #f9f2cb; }
#rfm69 td.data.empty { background: linear-gradient(to right, #e79b9b 0%, #e79b9b 45%, #fff 45%, #fff 55%, #f9f2cb 55%); }
</style>

<table id="rfm69" cellpadding="10px">
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

  <td class="datapacket" colspan=2>type | len1 <span>1byte</span></td>
  <td class="datapacket" colspan=3>data <span>len1 bytes</span></td>

  <td class="datapacket" colspan=2>type | len2 <span>1byte</span></td>
  <td class="datapacket" colspan=3>data <span>len2 bytes</span></td>
  <td class="datapacket empty">...</td>
</tr>
<tr class="hidden">
  <td class="hidden" colspan=4></td>

  <td class="typelen">type <span>4bits</span>
  <td class="typelen">len1 <span>4bits</span>

  <td class="data">data <span>1byte</span></td>
  <td class="data">data <span>1byte</span></td>
  <td class="data">...</td>

  <td class="typelen">type <span>4bits</span>
  <td class="typelen">len2 <span>4bits</span>

  <td class="data">data <span>1byte</span></td>
  <td class="data">data <span>1byte</span></td>
  <td class="data">...</td>
  <td class="data empty">...</td>
</tr>

</table>
