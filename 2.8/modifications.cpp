optional<Edge*> HalfedgeMesh::flip_edge(Edge* e)
{
    if(e == nullptr || e->on_boundary()) {
        return std::nullopt;
    }

    //要用到的半边
    Halfedge*h =e->halfedge;
    Halfedge*h_inv =h->inv;
    Halfedge*h_2_3 =h->next;
    Halfedge*h_3_1 =h_2_3->next;
    Halfedge*h_1_4 =h_inv->next;
    Halfedge*h_4_2 =h_1_4->next;
    //要用到的顶点
    //v1 andv2 are verticesalongtheedge
    Vertex*v1 =h->from;
    Vertex*v2 =h_inv->from;
    //v3 andv4 are verticesopposite the edge
    Vertex*v3 =h_3_1->from;
    Vertex*v4 =h_4_2->from;

    //要用到的面片
    Face*f1 =h->face;
    Face*f2 =h_inv->face;

    //创建
    Halfedge* h_4_3 = h;
    Halfedge* h_3_4 = h_inv;

    //设置半边
    h_4_3->set_neighbors(h_3_1, h_1_4, h_3_4, v4, e, f1);
    h_3_4->set_neighbors(h_4_2, h_2_3, h_4_3, v3, e, f2);

    h_3_1->set_neighbors(h_1_4, h_4_3, h_3_1->inv, v3, h_3_1->edge, f1);
    h_1_4->set_neighbors(h_4_3, h_3_1, h_1_4->inv, v1, h_1_4->edge, f1);
    h_4_2->set_neighbors(h_2_3, h_3_4, h_4_2->inv, v4, h_4_2->edge, f2);
    h_2_3->set_neighbors(h_3_4, h_4_2, h_2_3->inv, v2, h_2_3->edge, f2);

    //设置顶点
    v1->halfedge = h_1_4;
    v2->halfedge = h_2_3;
    v3->halfedge = h_3_1;
    v4->halfedge = h_4_2;

    //设置面片
    f2->halfedge = h_3_4;
    f1->halfedge = h_4_3;

    //设置边
    e->halfedge = h_3_4;

    return e;
}

optional<Vertex*> HalfedgeMesh::split_edge(Edge* e)
{
    //build
    Vertex* new_v = new_vertex();
    Edge* new_e4 = new_edge();
    Edge* new_e2 = new_edge();
    Edge* new_e3 = new_edge();

    Face* f_n23 = new_face();
    Face* f_n14 = new_face();

    Halfedge* h_n3 = new_halfedge();
    Halfedge* h_3n = new_halfedge();
    Halfedge* h_n2 = new_halfedge();
    Halfedge* h_n1 = new_halfedge();
    Halfedge* h_n4 = new_halfedge();
    Halfedge* h_4n = new_halfedge();

    //existing elements
    Halfedge* h_1n = e->halfedge;
    Halfedge* h_2n = h_1n->inv;
    Halfedge* h_23 = h_1n->next;
    Halfedge* h_31 = h_23->next;
    Halfedge* h_14 = h_2n->next;
    Halfedge* h_42 = h_14->next;
    Face* f_1n3 = h_1n->face;
    Face* f_2n4 = h_2n->face;
    Vertex* v1 = h_1n->from;
    Vertex* v2 = h_2n->from;
    Vertex* v3 = h_31->from;
    Vertex* v4 = h_42->from;

    //modify halfedges
    h_1n->set_neighbors(h_n3, h_31, h_n1, v1, e, f_1n3);
    h_n3->set_neighbors(h_31, h_1n, h_3n, new_v, new_e3, f_1n3);
    h_31->set_neighbors(h_1n, h_n3, h_31->inv, v3, h_31->edge, f_1n3);

    h_n2->set_neighbors(h_23, h_3n, h_2n, new_v, new_e2, f_n23);
    h_23->set_neighbors(h_3n, h_n2, h_23->inv, v2, h_23->edge, f_n23);
    h_3n->set_neighbors(h_n2, h_23, h_n3, v3, new_e3, f_n23);

    h_n1->set_neighbors(h_14, h_4n, h_1n, new_v, e, f_n14);
    h_14->set_neighbors(h_4n, h_n1, h_14->inv, v1, h_14->edge, f_n14);
    h_4n->set_neighbors(h_n1, h_14, h_n4, v4, new_e4, f_n14);

    h_n4->set_neighbors(h_42, h_2n, h_4n, new_v, new_e4, f_2n4);
    h_42->set_neighbors(h_2n, h_n4, h_42->inv, v4, h_42->edge, f_2n4);
    h_2n->set_neighbors(h_n4, h_42, h_n2, v2, new_e2, f_2n4);

    //modify vertices
    v1->halfedge = h_14;
    v2->halfedge = h_23;
    v3->halfedge = h_31;
    v4->halfedge = h_42;
    new_v->halfedge = h_n3;
    new_v->pos = new_v->neighborhood_center();
    new_v->is_new = true;

    //modify faces
    f_1n3->halfedge = h_1n;
    f_n23->halfedge = h_n2;
    f_2n4->halfedge = h_n4;
    f_n14->halfedge = h_n1;

    //modify edges
    e->halfedge = h_1n;
    new_e2->halfedge = h_n2;
    new_e3->halfedge = h_n3;
    new_e4->halfedge = h_n4;

    /*
    {
        logger->trace("---start spliting edge {}---",e->id);
        //假如将e的两个端点赋值给v1和v2，将两个相对位置的点赋值给v3和v4
        logger->trace("(v1, v2)({},{})",v1->id,v2->id);
        logger->trace("(v3, v4)({},{})",v3->id,v4->id);
        //修改各种指针
        logger->trace("face 1234:{}->{}->{}",f_1n3->halfedge->from->id,
        f1->halfedge->next->from->id,
        f1->halfedge->next->next->from->id);
        logger->trace("face 214:{}->{}->{}",f2->halfedge->from->id,
        f2->halfedge->next->from->id,
        f2->halfedge->next->next->from->id);
        logger->trace("---end---");
    }*/
    return new_v;
}
optional<Vertex*> HalfedgeMesh::collapse_edge(Edge* e)
{
    if (e == nullptr) {
        return std::nullopt;
    }

    Halfedge* h12 = e->halfedge;
    Halfedge* h1next = h12->next;
    Halfedge* h1prev = h12->prev;
    Halfedge* h21 = h12->inv;
    Halfedge* h2next = h21->next;
    Halfedge* h2prev = h21->prev;

    Halfedge* h1 = h1prev->inv;
    Halfedge* hinv1 = h1next->inv;
    Halfedge* h2 = h2prev->inv;
    Halfedge* hinv2 = h2next->inv;

    Vertex* v1 = h1->from;
    Vertex* v2 = h21->from;
    Vertex* v3 = hinv1->from;
    Vertex* v4 = hinv2->from;

    Edge* e1 = h1->edge;
    Edge* e2 = hinv1->edge;
    Edge* e3 = hinv2->edge;
    Edge* e4 = h2->edge;

    Face* f1 = h1->face;
    Face* f2 = h2->face;
    Face* e_f1 = h12->face;
    Face* e_f2 = h21->face;

    Halfedge* h = v2->halfedge;
    do {
        h->from = v1;
        h = h->inv->next;
    }while(h != v2->halfedge);

    // 更新半边连接关系
    h1->set_neighbors(h1->next, h1->prev, hinv1, v1, e1, f1);
    hinv1->set_neighbors(hinv1->next, hinv1->prev, h1, v3, e1, f1);
    h2->set_neighbors(h2->next, h2->prev, hinv2, v1, e3, f2);
    hinv2->set_neighbors(hinv2->next, hinv2->prev, h2, v4, e3, f2);

    v1->halfedge = h1;
    v3->halfedge = hinv1;
    v4->halfedge = hinv2;
    v1->pos = v1->neighborhood_center();
    v3->pos = v3->neighborhood_center();
    v4->pos = v4->neighborhood_center();

    // 删除顶点、半边和边
    erase(v2);
    erase(h1prev);
    erase(h12);
    erase(h1next);
    erase(h2prev);
    erase(h21);
    erase(h2next);

    // 处理边界面
    if (h12->is_boundary() || h21->is_boundary()) {
        // 对边界的处理：保持虚拟面片不被删除
        if (e_f1 && !e_f1->is_boundary) {
            erase(e_f1); // 删除实际面片
        }
        if (e_f2 && !e_f2->is_boundary) {
            erase(e_f2); // 删除实际面片
        }
    } else {
        // 对非边界的面片删除
        erase(e_f1);
        erase(e_f2);
    }

    erase(e2);
    erase(e4);
    erase(e);

    return v1;
}

