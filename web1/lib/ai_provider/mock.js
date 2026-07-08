/**
 * mock provider — 不调真实 API,基于当前指标模板化输出报告
 *   目的:让用户没配 API key 也能看到完整 UI,方便调试前端
 */

function levelOf(good, ok) {
  return (v, thGood, thOk) => {
    if (good(v, thGood)) return 'low';
    if (ok(v, thOk)) return 'medium';
    return 'high';
  };
}

function summarizeMetrics(metricsSeries) {
  if (!metricsSeries || metricsSeries.length === 0) {
    return { avgSpm: 0, avgSym: 0, avgKneeL: 0, avgKneeR: 0, avgStab: 0, samples: 0 };
  }
  const avg = (k) => Math.round(metricsSeries.reduce((s, m) => s + (m[k] ?? 0), 0) / metricsSeries.length);
  return {
    avgSpm: avg('spm'),
    avgSym: avg('symmetry'),
    avgKneeL: avg('kneeAmpL'),
    avgKneeR: avg('kneeAmpR'),
    avgStab: avg('stability'),
    samples: metricsSeries.length,
  };
}

async function analyzeGait({ metricsSeries, windowMinutes }) {
  const s = summarizeMetrics(metricsSeries);

  // 风险等级判定
  const risks = [
    {
      name: '步态节律',
      level: (s.avgSpm >= 80 && s.avgSpm <= 120) ? 'low' : (s.avgSpm >= 60 && s.avgSpm <= 140) ? 'medium' : 'high',
      evidence: `窗口内平均步频 ${s.avgSpm} spm(正常成人 100-120)`,
    },
    {
      name: '左右对称性',
      level: s.avgSym >= 80 ? 'low' : s.avgSym >= 60 ? 'medium' : 'high',
      evidence: `平均对称性指数 ${s.avgSym}%(基于左右踝 x 摆幅比值)`,
    },
    {
      name: '膝关节活动度',
      level: (Math.min(s.avgKneeL, s.avgKneeR) >= 30) ? 'low'
           : (Math.min(s.avgKneeL, s.avgKneeR) >= 15) ? 'medium' : 'high',
      evidence: `左膝屈伸 ${s.avgKneeL}° / 右膝 ${s.avgKneeR}°(健康行走通常 40-60°)`,
    },
    {
      name: '平衡稳定性',
      level: s.avgStab >= 75 ? 'low' : s.avgStab >= 50 ? 'medium' : 'high',
      evidence: `平均稳定性 ${s.avgStab}(基于踝 y 相邻帧差分标准差反推)`,
    },
  ];

  // 动态建议
  const suggestions = [];
  if (s.avgSpm < 80) suggestions.push('步频偏慢,可尝试节拍器辅助训练(建议 100-110 bpm),配合助行器保持稳定推进节奏');
  else if (s.avgSpm > 130) suggestions.push('步频偏快,可能有代偿性小碎步,建议放慢节奏并增大步幅');
  if (s.avgSym < 70) suggestions.push('左右步态存在明显偏侧,建议评估单侧下肢肌力和感觉,进行对称性训练');
  if (Math.min(s.avgKneeL, s.avgKneeR) < 20) suggestions.push('膝关节屈伸幅度偏低,可能存在僵硬,建议每日进行踩单车或坐位膝屈伸拉伸');
  if (s.avgStab < 60) suggestions.push('平衡稳定性下降,建议增加核心训练和单脚站立练习,并确保助行器高度合适');
  if (suggestions.length === 0) suggestions.push('当前步态各项指标基本正常,建议保持规律行走训练,每次 15-20 分钟');
  suggestions.push('建议定期(每周)对比历史步态曲线,观察指标的长期趋势变化');

  const summary = `过去 ${windowMinutes} 分钟共 ${s.samples} 条指标采样。平均步频 ${s.avgSpm} spm,对称性 ${s.avgSym}%,膝屈伸 ${s.avgKneeL}°/${s.avgKneeR}°,稳定性 ${s.avgStab}。` +
    (risks.filter(r => r.level === 'high').length > 0
      ? `识别到 ${risks.filter(r => r.level === 'high').length} 项高风险指标,建议关注对应项目并结合专业评估。`
      : `总体步态指标处于可接受范围内,继续保持训练。`);

  const full = [
    '## 综合评估',
    '',
    summary,
    '',
    '## 逐项指标解读',
    '',
    `- **步频 (spm)** — 平均 ${s.avgSpm}。健康成人 100-120 spm 为典型范围,助行器使用者可略低。步频过低往往伴随迈步犹豫,过高可能是小碎步代偿。`,
    `- **左右对称性** — 平均 ${s.avgSym}%。< 70% 提示明显偏侧步态,常见于单侧下肢肌力减弱、卒中后遗症、髋/膝痛。`,
    `- **膝关节屈伸幅度** — 左 ${s.avgKneeL}°,右 ${s.avgKneeR}°。健康步态摆动相膝屈曲约 60°。幅度偏低提示膝关节僵硬或步态"直腿走"。`,
    `- **平衡稳定性** — ${s.avgStab}。反映踝点垂直位移的规律性;数值越高步态越规律。数值突降往往对应跌倒风险上升。`,
    '',
    '## 改进建议',
    '',
    suggestions.map(x => `- ${x}`).join('\n'),
    '',
    '## 注意',
    '',
    '本报告由本地模板规则生成(未调用大模型)。真实场景请配置 `AI_PROVIDER=deepseek` 获得更丰富的临床解读。',
  ].join('\n');

  return {
    summary,
    full,
    risks,
    suggestions,
    disclaimer: '本分析基于 K230 视觉步态观测,不构成医学诊断。识别到的异常仅供参考,如有持续性问题请咨询专业康复治疗师或神经科医生。',
    provider: 'mock',
    model: 'template-v1',
  };
}

module.exports = { analyzeGait };
