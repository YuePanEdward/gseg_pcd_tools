function [dist, proj_pt] = projpoint2plane(ref_pln,pt)

t=ref_pln.nvec(1) * pt(1) +ref_pln.nvec(2) * pt(2) +ref_pln.nvec(3) * pt(3) +ref_pln.offset;
proj_pt(1) = pt(1) - ref_pln.nvec(1) * t;
proj_pt(2) = pt(2) - ref_pln.nvec(2) * t;
proj_pt(3) = pt(3) - ref_pln.nvec(3) * t;

dist = abs(t) / norm(ref_pln.nvec,2);

end
